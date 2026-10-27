(() => {
  "use strict";

  // GPU renderer for Pollen's full-resolution public simulation geometry. The
  // generated asset keeps the MJCF kinematic tree and the original STL triangles;
  // only hidden electronics are omitted. Rendering and pose calculation stay on the computer.
  const FORMAT_VERSION = 2;
  const DEFAULT_AZIMUTH = -0.7;
  const DEFAULT_ELEVATION = 0.32;
  const LOOK_AT_Z = 0.15;
  const WINDOW_METRES = 0.36;
  const GRID_STEP = 0.05;
  const GRID_REACH = 0.30;
  const HOME_JOINTS = Object.freeze([
    0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0, 0, 0,
    0, 0.0873, 0.4579, 0.0049, -0.4530,
  ]);

  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
  const normalized = (vector) => {
    const length = Math.sqrt(dot(vector, vector));
    return length < 1e-9 ? [0, 0, 0] : vector.map((value) => value / length);
  };
  const identityPose = () => ({
    r: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    t: [0, 0, 0],
  });

  function applyPose(pose, vertex) {
    return [
      pose.r[0][0] * vertex[0] + pose.r[0][1] * vertex[1] + pose.r[0][2] * vertex[2] + pose.t[0],
      pose.r[1][0] * vertex[0] + pose.r[1][1] * vertex[1] + pose.r[1][2] * vertex[2] + pose.t[1],
      pose.r[2][0] * vertex[0] + pose.r[2][1] * vertex[1] + pose.r[2][2] * vertex[2] + pose.t[2],
    ];
  }

  function thenPose(parent, local) {
    const rotation = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let row = 0; row < 3; row += 1) {
      for (let column = 0; column < 3; column += 1) {
        rotation[row][column] = parent.r[row][0] * local.r[0][column]
          + parent.r[row][1] * local.r[1][column]
          + parent.r[row][2] * local.r[2][column];
      }
    }
    return { r: rotation, t: applyPose(parent, local.t) };
  }

  function quaternionPose(quaternion, translation) {
    const magnitude = Math.sqrt(quaternion.reduce((sum, value) => sum + value * value, 0));
    if (magnitude < 1e-9) return { ...identityPose(), t: translation.slice() };
    const [w, x, y, z] = quaternion.map((value) => value / magnitude);
    return {
      r: [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
      ],
      t: translation.slice(),
    };
  }

  function axisPose(axisValue, angle) {
    const [x, y, z] = normalized(axisValue);
    const sine = Math.sin(angle);
    const cosine = Math.cos(angle);
    const inverse = 1 - cosine;
    return {
      r: [
        [cosine + x * x * inverse, x * y * inverse - z * sine, x * z * inverse + y * sine],
        [y * x * inverse + z * sine, cosine + y * y * inverse, y * z * inverse - x * sine],
        [z * x * inverse - y * sine, z * y * inverse + x * sine, cosine + z * z * inverse],
      ],
      t: [0, 0, 0],
    };
  }

  function matrixFromPose(pose) {
    // WebGL matrices are column-major.
    return new Float32Array([
      pose.r[0][0], pose.r[1][0], pose.r[2][0], 0,
      pose.r[0][1], pose.r[1][1], pose.r[2][1], 0,
      pose.r[0][2], pose.r[1][2], pose.r[2][2], 0,
      pose.t[0], pose.t[1], pose.t[2], 1,
    ]);
  }

  function lowerBoundZ(pose, bounds) {
    return pose.t[2] + pose.r[2].reduce((sum, coefficient, axis) => (
      sum + coefficient * (coefficient >= 0 ? bounds.min[axis] : bounds.max[axis])
    ), 0);
  }

  class Reader {
    constructor(buffer) {
      this.buffer = buffer;
      this.view = new DataView(buffer);
      this.at = 0;
    }

    ensure(length) {
      if (this.at + length > this.view.byteLength) throw new Error("高精度模型文件不完整");
    }

    bytes(length) {
      this.ensure(length);
      const value = new Uint8Array(this.buffer, this.at, length);
      this.at += length;
      return value;
    }

    u16() { this.ensure(2); const value = this.view.getUint16(this.at, true); this.at += 2; return value; }
    i16() { this.ensure(2); const value = this.view.getInt16(this.at, true); this.at += 2; return value; }
    u32() { this.ensure(4); const value = this.view.getUint32(this.at, true); this.at += 4; return value; }
    f32() { this.ensure(4); const value = this.view.getFloat32(this.at, true); this.at += 4; return value; }
    vec3() { return [this.f32(), this.f32(), this.f32()]; }
    quaternion() { return [this.f32(), this.f32(), this.f32(), this.f32()]; }

    float32Array(length) {
      this.ensure(length * 4);
      if (this.at % 4) throw new Error("高精度模型浮点数据未对齐");
      const value = new Float32Array(this.buffer, this.at, length);
      this.at += length * 4;
      return value;
    }

    uint32Array(length) {
      this.ensure(length * 4);
      if (this.at % 4) throw new Error("高精度模型索引数据未对齐");
      const value = new Uint32Array(this.buffer, this.at, length);
      this.at += length * 4;
      return value;
    }
  }

  function parseModel(buffer) {
    const reader = new Reader(buffer);
    const magic = String.fromCharCode(...reader.bytes(4));
    const version = reader.u32();
    if (magic !== "DHD2" || version !== FORMAT_VERSION) {
      throw new Error("高精度模型格式与当前工作台不匹配");
    }
    const meshCount = reader.u16();
    const bodyCount = reader.u16();
    const partCount = reader.u16();
    reader.u16(); // reserved alignment field
    if (!meshCount || meshCount > 128 || !bodyCount || bodyCount > 128 || partCount > 512) {
      throw new Error("高精度模型数量字段异常");
    }

    const meshes = [];
    for (let meshIndex = 0; meshIndex < meshCount; meshIndex += 1) {
      const vertexCount = reader.u32();
      const triangleCount = reader.u32();
      if (!vertexCount || vertexCount > 200000 || !triangleCount || triangleCount > 300000) {
        throw new Error("高精度模型网格尺寸异常");
      }
      const vertices = reader.float32Array(vertexCount * 3);
      const indices = reader.uint32Array(triangleCount * 3);
      const minimum = [Infinity, Infinity, Infinity];
      const maximum = [-Infinity, -Infinity, -Infinity];
      for (let index = 0; index < vertices.length; index += 3) {
        for (let axis = 0; axis < 3; axis += 1) {
          minimum[axis] = Math.min(minimum[axis], vertices[index + axis]);
          maximum[axis] = Math.max(maximum[axis], vertices[index + axis]);
        }
      }
      for (let index = 0; index < indices.length; index += 1) {
        if (indices[index] >= vertexCount) throw new Error("高精度模型索引越界");
      }
      meshes.push({ vertices, indices, triangleCount, bounds: { min: minimum, max: maximum } });
    }

    const bodies = Array.from({ length: bodyCount }, () => ({
      parent: reader.i16(),
      joint: reader.i16(),
      position: reader.vec3(),
      quaternion: reader.quaternion(),
      axis: reader.vec3(),
    }));
    const parts = Array.from({ length: partCount }, () => ({
      body: reader.u16(),
      mesh: reader.u16(),
      colour: Array.from(reader.bytes(4).slice(0, 3), (value) => value / 255),
      position: reader.vec3(),
      quaternion: reader.quaternion(),
    }));
    const valid = bodies.every((body, index) => body.parent < index && body.joint < 15)
      && parts.every((part) => part.body < bodies.length && part.mesh < meshes.length);
    if (!valid || reader.at !== reader.view.byteLength) throw new Error("高精度模型结构校验失败");
    return { meshes, bodies, parts };
  }

  function compileShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      const message = gl.getShaderInfoLog(shader) || "未知着色器错误";
      gl.deleteShader(shader);
      throw new Error(`三维着色器编译失败：${message}`);
    }
    return shader;
  }

  function createProgram(gl) {
    const vertex = compileShader(gl, gl.VERTEX_SHADER, `#version 300 es
      precision highp float;
      layout(location = 0) in vec3 aPosition;
      uniform mat4 uModel;
      uniform vec3 uRight;
      uniform vec3 uUp;
      uniform vec3 uForward;
      uniform vec2 uViewScale;
      uniform float uFloor;
      uniform float uLookAt;
      out vec3 vWorld;
      void main() {
        vec3 world = (uModel * vec4(aPosition, 1.0)).xyz;
        world.z -= uFloor;
        vWorld = world;
        gl_Position = vec4(
          dot(world, uRight) * uViewScale.x,
          (dot(world, uUp) - uLookAt) * uViewScale.y,
          dot(world, uForward) * 2.0,
          1.0
        );
      }
    `);
    const fragment = compileShader(gl, gl.FRAGMENT_SHADER, `#version 300 es
      precision highp float;
      uniform vec3 uColour;
      uniform vec3 uLight;
      uniform bool uLineMode;
      uniform float uOpacity;
      in vec3 vWorld;
      out vec4 outColour;
      void main() {
        if (uLineMode) {
          outColour = vec4(uColour, 1.0);
          return;
        }
        vec3 normal = normalize(cross(dFdx(vWorld), dFdy(vWorld)));
        if (!gl_FrontFacing) normal = -normal;
        float diffuse = max(dot(normal, normalize(uLight)), 0.0);
        float hemisphere = 0.5 + 0.5 * normal.z;
        float shade = 0.56 + 0.34 * diffuse + 0.10 * hemisphere;
        outColour = vec4(uColour * shade, uOpacity);
      }
    `);
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    gl.deleteShader(vertex);
    gl.deleteShader(fragment);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      const message = gl.getProgramInfoLog(program) || "未知链接错误";
      gl.deleteProgram(program);
      throw new Error(`三维着色器链接失败：${message}`);
    }
    return program;
  }

  class MicroduckPoseView {
    constructor(canvas, { maxFps = 25 } = {}) {
      this.canvas = canvas;
      this.gl = canvas.getContext("webgl2", {
        alpha: true,
        antialias: true,
        depth: true,
        premultipliedAlpha: false,
        powerPreference: "high-performance",
      });
      this.model = null;
      this.selectedPart = -1;
      this.joints = HOME_JOINTS.slice();
      this.azimuth = DEFAULT_AZIMUTH;
      this.elevation = DEFAULT_ELEVATION;
      this.pendingFrame = 0;
      this.lastDrawAt = 0;
      this.frameInterval = 1000 / Math.min(60, Math.max(1, maxFps));
      this.pendingTimer = null;
      this.drag = null;
      this.error = this.gl ? null : new Error("当前浏览器未提供 WebGL 2，无法显示高精度模型");
      if (this.gl) this.initialiseGraphics();
      this.installInteraction();
      this.resizeObserver = new ResizeObserver(() => this.invalidate(true));
      this.resizeObserver.observe(canvas);
      this.paintEmpty();
    }

    initialiseGraphics() {
      const gl = this.gl;
      this.program = createProgram(gl);
      this.uniforms = Object.fromEntries([
        "uModel", "uRight", "uUp", "uForward", "uViewScale", "uFloor",
        "uLookAt", "uColour", "uLight", "uLineMode", "uOpacity",
      ].map((name) => [name, gl.getUniformLocation(this.program, name)]));
      const grid = [];
      const lineCount = Math.round(GRID_REACH / GRID_STEP);
      for (let index = -lineCount; index <= lineCount; index += 1) {
        const coordinate = index * GRID_STEP;
        grid.push(coordinate, -GRID_REACH, -0.0008, coordinate, GRID_REACH, -0.0008);
        grid.push(-GRID_REACH, coordinate, -0.0008, GRID_REACH, coordinate, -0.0008);
      }
      this.gridVertexCount = grid.length / 3;
      this.gridVao = gl.createVertexArray();
      this.gridBuffer = gl.createBuffer();
      gl.bindVertexArray(this.gridVao);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.gridBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(grid), gl.STATIC_DRAW);
      gl.enableVertexAttribArray(0);
      gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
      gl.bindVertexArray(null);
      gl.enable(gl.DEPTH_TEST);
      gl.depthFunc(gl.LEQUAL);
      gl.disable(gl.CULL_FACE);
    }

    async load(url) {
      if (this.error) throw this.error;
      const response = await fetch(url, { cache: "no-store" });
      if (!response.ok) throw new Error(`高精度模型加载失败（HTTP ${response.status}）`);
      this.model = parseModel(await response.arrayBuffer());
      const gl = this.gl;
      this.model.meshes.forEach((mesh) => {
        mesh.vao = gl.createVertexArray();
        mesh.vertexBuffer = gl.createBuffer();
        mesh.indexBuffer = gl.createBuffer();
        gl.bindVertexArray(mesh.vao);
        gl.bindBuffer(gl.ARRAY_BUFFER, mesh.vertexBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, mesh.vertices, gl.STATIC_DRAW);
        gl.enableVertexAttribArray(0);
        gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, mesh.indexBuffer);
        gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, mesh.indices, gl.STATIC_DRAW);
      });
      gl.bindVertexArray(null);
      this.invalidate(true);
      return this.model;
    }

    setState(joints) {
      this.joints = Array.from({ length: 15 }, (_, index) => {
        const value = Number(joints[index]);
        return Number.isFinite(value) ? value : 0;
      });
      this.invalidate();
    }

    setSelectedPart(index) {
      const selected = Number.isInteger(index) && index >= 0 ? index : -1;
      if (selected === this.selectedPart) return;
      this.selectedPart = selected;
      this.invalidate(true);
    }

    resetView() {
      this.azimuth = DEFAULT_AZIMUTH;
      this.elevation = DEFAULT_ELEVATION;
      this.invalidate(true);
    }

    installInteraction() {
      this.canvas.addEventListener("pointerdown", (event) => {
        this.drag = { x: event.clientX, y: event.clientY };
        this.canvas.setPointerCapture(event.pointerId);
      });
      this.canvas.addEventListener("pointermove", (event) => {
        if (!this.drag) return;
        const deltaX = event.clientX - this.drag.x;
        const deltaY = event.clientY - this.drag.y;
        this.drag = { x: event.clientX, y: event.clientY };
        this.azimuth += deltaX * 0.012;
        this.elevation = Math.max(0.05, Math.min(0.82, this.elevation - deltaY * 0.008));
        this.invalidate(true);
      });
      const endDrag = () => { this.drag = null; };
      this.canvas.addEventListener("pointerup", endDrag);
      this.canvas.addEventListener("pointercancel", endDrag);
      this.canvas.addEventListener("dblclick", () => this.resetView());
    }

    invalidate(immediate = false) {
      if (this.pendingFrame || !this.gl) return;
      if (this.pendingTimer !== null) {
        if (!immediate) return;
        clearTimeout(this.pendingTimer);
        this.pendingTimer = null;
      }
      const elapsed = performance.now() - this.lastDrawAt;
      const delay = immediate ? 0 : Math.max(0, this.frameInterval - elapsed);
      const schedule = () => {
        this.pendingTimer = null;
        this.pendingFrame = requestAnimationFrame(() => {
          this.pendingFrame = 0;
          this.render();
        });
      };
      if (delay) this.pendingTimer = setTimeout(schedule, delay);
      else schedule();
    }

    resizeCanvas() {
      const pixelRatio = Math.min(window.devicePixelRatio || 1, 1.6);
      const width = Math.max(1, Math.round(this.canvas.clientWidth * pixelRatio));
      const height = Math.max(1, Math.round(this.canvas.clientHeight * pixelRatio));
      if (this.canvas.width !== width || this.canvas.height !== height) {
        this.canvas.width = width;
        this.canvas.height = height;
      }
      return { width, height };
    }

    paintEmpty() {
      if (!this.gl) return;
      const { width, height } = this.resizeCanvas();
      this.gl.viewport(0, 0, width, height);
      this.gl.clearColor(0, 0, 0, 0);
      this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.DEPTH_BUFFER_BIT);
    }

    render() {
      this.lastDrawAt = performance.now();
      const gl = this.gl;
      const { width, height } = this.resizeCanvas();
      gl.viewport(0, 0, width, height);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      if (!this.model || width < 20 || height < 20) return;

      const bodyPoses = [];
      this.model.bodies.forEach((body) => {
        const parent = body.parent < 0 ? identityPose() : bodyPoses[body.parent];
        let pose = thenPose(parent, quaternionPose(body.quaternion, body.position));
        if (body.joint >= 0) pose = thenPose(pose, axisPose(body.axis, this.joints[body.joint] || 0));
        bodyPoses.push(pose);
      });
      const partPoses = this.model.parts.map((part) => (
        thenPose(bodyPoses[part.body], quaternionPose(part.quaternion, part.position))
      ));
      const floor = this.model.parts.reduce((lowest, part, index) => (
        Math.min(lowest, lowerBoundZ(partPoses[index], this.model.meshes[part.mesh].bounds))
      ), Infinity);
      if (!Number.isFinite(floor)) return;

      const sineAzimuth = Math.sin(this.azimuth);
      const cosineAzimuth = Math.cos(this.azimuth);
      const sineElevation = Math.sin(this.elevation);
      const cosineElevation = Math.cos(this.elevation);
      const forward = [
        -cosineElevation * cosineAzimuth,
        -cosineElevation * sineAzimuth,
        -sineElevation,
      ];
      const right = normalized(cross(forward, [0, 0, 1]));
      const up = cross(right, forward);
      const light = normalized([
        0.35 * right[0] + 0.72 * up[0] - 0.62 * forward[0],
        0.35 * right[1] + 0.72 * up[1] - 0.62 * forward[1],
        0.35 * right[2] + 0.72 * up[2] - 0.62 * forward[2],
      ]);
      const pixelScale = Math.min(width / WINDOW_METRES, height / WINDOW_METRES);
      const viewScale = [2 * pixelScale / width, 2 * pixelScale / height];
      const lookAt = dot([0, 0, LOOK_AT_Z], up);

      gl.useProgram(this.program);
      gl.uniform1f(this.uniforms.uOpacity, 1);
      gl.uniform3fv(this.uniforms.uRight, right);
      gl.uniform3fv(this.uniforms.uUp, up);
      gl.uniform3fv(this.uniforms.uForward, forward);
      gl.uniform2fv(this.uniforms.uViewScale, viewScale);
      gl.uniform1f(this.uniforms.uLookAt, lookAt);
      gl.uniform3fv(this.uniforms.uLight, light);

      gl.uniformMatrix4fv(this.uniforms.uModel, false, matrixFromPose(identityPose()));
      gl.uniform1f(this.uniforms.uFloor, 0);
      gl.uniform3fv(this.uniforms.uColour, [0.89, 0.91, 0.93]);
      gl.uniform1i(this.uniforms.uLineMode, 1);
      gl.bindVertexArray(this.gridVao);
      gl.drawArrays(gl.LINES, 0, this.gridVertexCount);

      gl.uniform1f(this.uniforms.uFloor, floor);
      gl.uniform1i(this.uniforms.uLineMode, 0);
      this.model.parts.forEach((part, index) => {
        const mesh = this.model.meshes[part.mesh];
        gl.uniformMatrix4fv(this.uniforms.uModel, false, matrixFromPose(partPoses[index]));
        const selected = index === this.selectedPart;
        gl.uniform3fv(this.uniforms.uColour, selected ? [0.12, 0.60, 1.0] : part.colour);
        gl.bindVertexArray(mesh.vao);
        gl.drawElements(gl.TRIANGLES, mesh.indices.length, gl.UNSIGNED_INT, 0);
      });
      // Draw the selected housing's hidden surfaces over the shell. Keep the
      // scene depth intact so visible surfaces retain their normal shading.
      const selectedPart = this.model.parts[this.selectedPart];
      if (selectedPart) {
        const mesh = this.model.meshes[selectedPart.mesh];
        gl.uniformMatrix4fv(this.uniforms.uModel, false, matrixFromPose(partPoses[this.selectedPart]));
        gl.uniform3fv(this.uniforms.uColour, [0.12, 0.60, 1.0]);
        gl.uniform1f(this.uniforms.uOpacity, 0.45);
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        gl.depthFunc(gl.GREATER);
        gl.depthMask(false);
        gl.bindVertexArray(mesh.vao);
        gl.drawElements(gl.TRIANGLES, mesh.indices.length, gl.UNSIGNED_INT, 0);
        gl.depthMask(true);
        gl.depthFunc(gl.LEQUAL);
        gl.disable(gl.BLEND);
        gl.uniform1f(this.uniforms.uOpacity, 1);
      }
      gl.bindVertexArray(null);
    }
  }

  MicroduckPoseView.HOME_JOINTS = HOME_JOINTS;
  window.MicroduckPoseView = MicroduckPoseView;
})();
