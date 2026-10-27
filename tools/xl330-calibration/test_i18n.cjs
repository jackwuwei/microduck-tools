'use strict';
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, 'i18n.js'), 'utf8');
function setup({saved, browser='zh-CN', languages, blocked=false}={}) {
  const nodes = [];
  const attributes = [];
  const selector = {value:''};
  const storage = new Map(saved ? [['microduck-language',saved]] : []);
  const document = {
    documentElement:{lang:''},
    createTreeWalker(){let index=-1;return {nextNode(){this.currentNode=nodes[++index];return !!this.currentNode;}};},
    querySelectorAll(){return attributes;},
    getElementById(){return selector;},
  };
  const context={window:{},document,navigator:{language:browser,languages},NodeFilter:{SHOW_TEXT:4},
    localStorage:{getItem(key){if(blocked)throw Error('blocked');return storage.get(key);},
      setItem(key,value){if(blocked)throw Error('blocked');storage.set(key,value);}}};
  vm.runInNewContext(source,context);
  return {i18n:context.window.MicroduckI18n,nodes,attributes,document,storage,selector};
}
const text = value => ({textContent:value,parentElement:{tagName:'P'}});

test('language follows browser initially and saved preference on reload',()=>{
  assert.equal(setup({browser:'en-US'}).i18n.language,'en');
  assert.equal(setup({browser:'zh-TW'}).i18n.language,'zh');
  const session=setup();session.i18n.setLanguage('en');
  assert.equal(session.storage.get('microduck-language'),'en');
  assert.equal(setup({saved:session.storage.get('microduck-language')}).i18n.language,'en');
  assert.equal(setup({browser:'en',blocked:true}).i18n.language,'en');
});

test('default respects the first browser language and handles missing or invalid preferences',()=>{
  assert.equal(setup({languages:['en-US','zh-CN']}).i18n.language,'en');
  assert.equal(setup({browser:'en-US',languages:['zh-TW','en-US']}).i18n.language,'zh');
  assert.equal(setup({languages:['ZH-hant']}).i18n.language,'zh');
  assert.equal(setup({languages:['fr-FR','zh-CN']}).i18n.language,'en');
  assert.equal(setup({browser:'en-US',languages:[]}).i18n.language,'en');
  assert.equal(setup({browser:null}).i18n.language,'en');
  assert.equal(setup({saved:'invalid',browser:'en-US'}).i18n.language,'en');
  assert.equal(setup({saved:'zh',languages:['en-US']}).i18n.language,'zh');
  assert.equal(setup({blocked:true,languages:['en-US']}).i18n.language,'en');
});

test('switching restores original static and dynamic text without changing readings',()=>{
  const s=setup();const status=text('已连接 · 全部松力'),capture=text('零位：2051 pulse');
  s.nodes.push(status,capture,text('ID 23 · 左腿 · 膝盖'));
  const aria={dataset:{i18nAria:'历史备份'},setAttribute(key,value){this[key]=value;}};
  s.attributes.push(aria);
  s.i18n.setLanguage('en');
  assert.equal(status.textContent,'Connected · all torque off');
  assert.equal(capture.textContent,'Zero：2051 pulse');
  assert.equal(s.nodes[2].textContent,'ID 23 · Left leg · Knee');
  assert.equal(aria['aria-label'],'Backup history');
  assert.equal(s.document.documentElement.lang,'en');
  capture.textContent='零位：2052 pulse';s.i18n.apply();
  assert.equal(capture.textContent,'Zero：2052 pulse');
  s.i18n.setLanguage('zh');
  assert.equal(capture.textContent,'零位：2052 pulse');
  assert.equal(status.textContent,'已连接 · 全部松力');
  assert.equal(aria['aria-label'],'历史备份');
  assert.equal(s.document.documentElement.lang,'zh-CN');
});

test('all static Chinese page copy and accessibility labels have English translations',()=>{
  const {i18n}=setup({browser:'en'});
  const html=fs.readFileSync(path.join(__dirname,'index.html'),'utf8');
  const strings=[...html.matchAll(/>([^<>]+)</g)].map(m=>m[1]);
  strings.push(...[...html.matchAll(/data-i18n-aria="([^"]+)"/g)].map(m=>m[1]));
  for(const value of strings){
    if(value==='中文')continue;
    if(/[\u4e00-\u9fff]/.test(value))assert(!/[\u4e00-\u9fff]/.test(i18n.translate(value)),value);
  }
});

test('backend and model errors translate while preserving IDs and external diagnostics',()=>{
  const {i18n}=setup({browser:'en'});
  assert.equal(i18n.translate('ID 23：电压或温度不符合标定条件'),
    'ID 23：Voltage or temperature outside calibration conditions');
  assert.equal(i18n.translate('SSH 串口连接结束；请先在终端确认 ssh microduck 可登录。Permission denied'),
    'SSH serial connection ended; first verify ssh microduck in a terminal. Permission denied');
  assert.equal(i18n.translate('高精度模型加载失败（HTTP 404）'),
    'High-resolution model loading failed（HTTP 404）');
  assert.equal(i18n.translate('所选 +10°'),'Selected +10°');
});
