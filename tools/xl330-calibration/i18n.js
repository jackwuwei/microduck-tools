(() => {
  'use strict';
  const english = {
  "Microduck · XL330 标定": "Microduck · XL330 Calibration",
  "DYNAMIXEL XL330 标定": "DYNAMIXEL XL330 Calibration",
  "加载中": "Loading",
  "未连接": "Disconnected",
  "断开": "Disconnect",
  "选择关节": "Select joint",
  "左右按鸭子自身。逐个完成后切换下一个。": "Left and right are from the robot’s perspective. Finish one joint before moving on.",
  "姿态参考": "Pose reference",
  "零位 0°": "Zero 0°",
  "所选 +10°": "Selected +10°",
  "所选 −10°": "Selected −10°",
  "HOME 站姿": "HOME standing pose",
  "当前编码器": "Current encoder",
  "模型参考，不发送运动指令。": "Model reference only. No motion commands are sent.",
  "拖动旋转，双击复位。嘴部模型固定，ID 34 请以实物及编码器判断。HOME 是站姿参考，不能当作所有关节的零位。": "Drag to rotate; double-click to reset. The beak model is fixed: use the physical beak and encoder for ID 34. HOME is a standing reference, not the zero pose for all joints.",
  "连接与备份": "Connection and backups",
  "仅支持 XL330-M288（型号 1200）、位置模式 3、15 个官方 ID。连接后自动备份原参数。": "Supports the 15 official IDs, XL330-M288 (model 1200), position mode 3 only. Original parameters are backed up automatically on connection.",
  "已支撑机器人、全部舵机松力，主控和其他总线程序已停止": "The robot is supported, all servos have torque off, and the controller and other bus programs are stopped",
  "检测并连接": "Check and connect",
  "程序仅写方向、零位偏移与位置限位；不启用扭矩、不发送目标位置。断开后参数仍保存在舵机中。": "Only direction, homing offset and position limits are written. Torque is never enabled and no position targets are sent. Servo parameters persist after disconnection.",
  "等待连接": "Waiting for connection",
  "演示：模拟手动摆动": "Demo: simulate manual movement",
  "1 · 核对方向": "1 · Check direction",
  "查看 +10° 模型，手动向同方向轻推。编码器角度应增加；若减少，先核对 ID、视角和装配。软件补偿流程不直接改方向位。": "View the +10° model and gently move the joint in that direction by hand. The encoder angle should increase. If it decreases, check the ID, view and assembly. The software trim workflow does not change the direction bit.",
  "设置正向": "Set normal direction",
  "设置反向": "Set reversed direction",
  "这两个按钮会立即写入 Drive Mode 的方向位，并回读校验。": "These buttons immediately write the Drive Mode direction bit and verify it by reading back.",
  "2 · 采集零位与限位": "2 · Capture zero and limits",
  "软件补偿流程只点“采集当前为 0°”，确认采集框和左侧零✓后记录。仅在另行使用 EEPROM 标定时采两个安全限位。": "For software trim, use only “Capture current as 0°”, verify the captured value and Zero ✓ in the list, then record it. Capture the two safe limits only for the separate EEPROM calibration workflow.",
  "采集当前为 0°": "Capture current as 0°",
  "采集负向安全限位": "Capture safe negative limit",
  "采集正向安全限位": "Capture safe positive limit",
  "尚未采集": "No samples captured",
  "3 · EEPROM 写入（可选）": "3 · Write EEPROM (optional)",
  "软件零位补偿流程不使用此步骤；采样结果需另行记录并应用到 robotd。": "Skip this step for software zero trim. Record the samples separately and apply them to robotd.",
  "计算寄存器变化": "Preview register changes",
  "已核对方向、零位和安全限位": "I have checked direction, zero and safe limits",
  "写入所选关节并校验": "Write selected joint and verify",
  "恢复所选关节的连接前参数": "Restore selected joint’s pre-connection parameters",
  "若写入中断，重连后可选择之前的备份恢复。请确认备份来自同一只机器人及同一批舵机。": "If a write is interrupted, reconnect and choose an earlier backup to restore. Confirm it belongs to this robot and these servos.",
  "从所选备份恢复当前关节": "Restore current joint from selected backup",
  "关节姿态参考模型": "Joint pose reference model",
  "历史备份": "Backup history",
  "演示模式 · 无硬件连接": "Demo mode · no hardware connection",
  "真机模式": "Hardware mode",
  "电脑串口": "Local serial port",
  "串口": "Serial port",
  "EEPROM 写入": "EEPROM writes",
  "已连接 · 全部松力": "Connected · all torque off",
  "连接前备份：": "Pre-connection backup: ",
  "3D 不可用：": "3D unavailable: ",
  "；请先修复模型显示再核对实物。": "; fix the model display before checking the robot.",
  "当前编码器：使用舵机已保存的零位与方向；尚未应用采集草稿。躯干按水平显示。": "Current encoder: uses the servo’s saved offset and direction; draft samples are not applied. The torso is shown level.",
  "当前编码器不可用；模型暂按零位示意。": "Current encoder unavailable; showing the zero pose for reference.",
  "模型参考：仅改变屏幕姿态，不控制实物；采集 0° 时请使用“零位 0°”。": "Model reference: changes only the screen pose, without moving the robot. Use “Zero 0°” when capturing zero.",
  "输入电压报警": "Input voltage alert",
  "本次已写入并回读校验": "Written and read-back verified in this session",
  "已写入": "Written",
  "硬件状态": "Hardware status",
  "负限位": "Negative limit",
  "正限位": "Positive limit",
  "零位": "Zero",
  "方向": "Direction",
  "反向": "Reversed",
  "正向": "Normal",
  "偏移": "Offset",
  "限位": "Limits",
  "寄存器": "Register",
  "待写入": "Proposed",
  "当前": "Current",
  "恢复": "Restore",
  "关节": "Joint",
  "左腿": "Left leg",
  "右腿": "Right leg",
  "头颈": "Neck/head",
  "髋部转向": "Hip yaw",
  "髋部侧摆": "Hip roll",
  "髋部前后": "Hip pitch",
  "膝盖": "Knee",
  "脚踝": "Ankle",
  "颈部俯仰": "Neck pitch",
  "头部俯仰": "Head pitch",
  "转向": "Yaw",
  "侧摆": "Roll",
  "鸭嘴": "Beak",
  "请求失败": "Request failed",
  "。会话已停止；请检查后重新连接。": ". Session stopped; check the issue before reconnecting.",
  "反馈已过期": "Feedback stale",
  "读数过期": "Readings stale",
  "零": "Zero",
  "负": "Neg",
  "正": "Pos",
  "先采集零位和两个限位": "Capture zero and both limits first",
  "编码器处于跨圈坐标；请断开工具，正常断电重启舵机后重连并重新采集": "Encoder is in multi-turn coordinates; disconnect, power-cycle the servos normally, reconnect and recapture",
  "零位需要的偏移超出 ±1024；请调整舵盘安装位置后重新采集": "Required zero offset exceeds ±1024; reinstall the horn and recapture",
  "限位必须位于 0–4095 且包围零位；请按模型负向/正向重新采集，勿跨整圈": "Limits must be within 0–4095 and surround zero; recapture in the model’s negative/positive directions without crossing a full turn",
  "无法打开串口": "Cannot open serial port",
  "总线已有数据；请先停止 robotd、观察台和其他串口程序": "Bus traffic detected; stop robotd, the observer and other serial programs first",
  "DYNAMIXEL 通信失败：": "DYNAMIXEL communication failed: ",
  "发现其他总线流量，停止会话": "Other bus traffic detected; session stopped",
  "该寄存器禁止写入": "Writing this register is prohibited",
  "回读不一致": "Read-back mismatch",
  "请先连接": "Connect first",
  "仅支持 XL330-M288、位置模式 3；不自动切换模式": "Only XL330-M288 in position mode 3 is supported; mode is never changed automatically",
  "必须扭矩关闭、硬件错误为零": "Torque must be off and hardware errors clear (the Microduck 2S profile permits input-voltage alert 0x01)",
  "电压或温度不符合标定条件": "Voltage or temperature outside calibration conditions",
  "参数被其他程序修改；请断开后重新连接": "Parameters changed by another program; disconnect and reconnect",
  "已经连接": "Already connected",
  "未知采集类型": "Unknown capture type",
  "关节尚未静止，请稳定后重试": "Joint is still moving; hold it steady and retry",
  "方向必须为布尔值": "Direction must be a boolean",
  "最终参数回读失败": "Final parameter read-back failed",
  "预览已过期，请重新预览": "Preview expired; preview again",
  "无效备份路径": "Invalid backup path",
  "备份版本或模式不匹配": "Backup version or mode mismatch",
  "备份型号或模式不匹配": "Backup model or mode mismatch",
  "备份寄存器值无效": "Invalid backup register values",
  "仅允许本机访问": "Local access only",
  "未找到": "Not found",
  "需要同源请求": "Same-origin request required",
  "请求大小无效": "Invalid request size",
  "请求格式无效": "Invalid request format",
  "无效关节 ID": "Invalid joint ID",
  "无效模拟位置": "Invalid demo position",
  "未知操作": "Unknown operation",
  "无效关节或寄存器": "Invalid joint or register",
  "该寄存器禁止写入或数值无效": "Register write prohibited or invalid value",
  "未知串口操作": "Unknown serial operation",
  "SSH 目标必须是主机名、SSH 别名或 user@host": "SSH target must be a hostname, SSH alias or user@host",
  "SSH 连接中断；请重新连接并核对备份": "SSH connection interrupted; reconnect and check the backup",
  "SSH 串口响应超时；已停止会话，请检查网络和主控": "SSH serial response timed out; session stopped. Check the network and controller",
  "SSH 串口连接结束；请先在终端确认 ssh microduck 可登录。": "SSH serial connection ended; first verify ssh microduck in a terminal. ",
  "SSH 串口响应过大": "SSH serial response too large",
  "主控串口：": "Controller serial port: ",
  "操作失败": "Operation failed",
  "SSH 串口响应无效": "Invalid SSH serial response",
  "无效关节或禁止写入的寄存器": "Invalid joint or prohibited register write",
  "高精度模型文件不完整": "Incomplete high-resolution model file",
  "高精度模型浮点数据未对齐": "Misaligned model floating-point data",
  "高精度模型索引数据未对齐": "Misaligned model index data",
  "高精度模型格式与当前观察台不匹配": "Model format does not match this viewer",
  "高精度模型数量字段异常": "Invalid model counts",
  "高精度模型网格尺寸异常": "Invalid model mesh size",
  "高精度模型索引越界": "Model index out of range",
  "高精度模型结构校验失败": "Model structure validation failed",
  "未知着色器错误": "Unknown shader error",
  "三维着色器编译失败：": "3D shader compilation failed: ",
  "未知链接错误": "Unknown shader link error",
  "三维着色器链接失败：": "3D shader linking failed: ",
  "当前浏览器未提供 WebGL 2，无法显示高精度模型": "This browser does not provide WebGL 2; the high-resolution model cannot be displayed",
  "高精度模型加载失败": "High-resolution model loading failed"
};
  const escapePattern = text => Array.from(text, character =>
    '.*+?^${}()|[]\\'.includes(character) ? '\\' + character : character).join('');
  const pattern = new RegExp(Object.keys(english).sort((a,b)=>b.length-a.length)
    .map(escapePattern).join('|'), 'g');
  // The first browser preference is authoritative; do not choose Chinese
  // merely because it appears later in the browser's language list.
  const browserLanguage = navigator.languages?.[0] || navigator.language || 'en';
  let language = /^zh(?:-|$)/i.test(browserLanguage) ? 'zh' : 'en';
  try {
    const saved = localStorage.getItem('microduck-language');
    if (saved === 'zh' || saved === 'en') language = saved;
  } catch (_) {}
  const originals = new WeakMap();
  function translate(text) { return language === 'en' ? text.replace(pattern, key=>english[key]) : text; }
  function update(node, key) {
    const value = node[key];
    let saved = originals.get(node);
    if (!saved || value !== saved.output) saved = {source:value};
    saved.output = translate(saved.source);
    node[key] = saved.output;
    originals.set(node, saved);
  }
  function apply() {
    document.documentElement.lang = language === 'en' ? 'en' : 'zh-CN';
    const walk = document.createTreeWalker(document.documentElement, NodeFilter.SHOW_TEXT);
    while (walk.nextNode()) {
      const node = walk.currentNode;
      if (!['SCRIPT','STYLE'].includes(node.parentElement?.tagName)) update(node, 'textContent');
    }
    document.querySelectorAll('[data-i18n-aria]').forEach(node=>{
      node.setAttribute('aria-label', translate(node.dataset.i18nAria));
    });
    const select = document.getElementById('language');
    if (select) select.value = language;
  }
  function setLanguage(value) {
    language = value === 'en' ? 'en' : 'zh';
    try { localStorage.setItem('microduck-language', language); } catch (_) {}
    apply();
  }
  window.MicroduckI18n = {translate, apply, setLanguage, get language(){return language;}};
})();
