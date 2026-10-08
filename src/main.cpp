/*
 * ESP32-CAM Calliper
 * -----------------------------------------------------------------------------
 *  - ESP32-CAM (AI-Thinker) phát WiFi (Access Point) + webserver
 *  - Stream video MJPEG lên trình duyệt
 *  - Slider trên web điều khiển động cơ servo (PWM)
 *  - Đọc giá trị thước kẹp điện tử (digital caliper) hiển thị lên web
 *
 *  Sơ đồ chân dùng thêm (ngoài chân camera của AI-Thinker):
 *    - Servo signal : GPIO 12
 *    - Caliper CLK  : GPIO 13   (ngắt - interrupt)
 *    - Caliper DATA : GPIO 15
 *
 *  LƯU Ý PHẦN CỨNG: thước kẹp điện tử giao tiếp ở mức ~1.5V, KHÔNG nối trực
 *  tiếp vào GPIO 3.3V. Cần mạch dịch mức (level shifter / transistor). Tuỳ
 *  model thước mà tín hiệu data có thể bị đảo -> chỉnh CALIPER_DATA_INVERTED.
 * -----------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "soc/soc.h"           // tắt brownout detector
#include "soc/rtc_cntl_reg.h"

// ============================ CẤU HÌNH ======================================
// --- Access Point ---
static const char *AP_SSID = "ESP32CAM-Calliper";
static const char *AP_PASS = "12345678"; // >= 8 ký tự

// --- Chân điều khiển ---
#define SERVO_PIN        12
#define CALIPER_CLK_PIN  13
#define CALIPER_DATA_PIN 15
#define CALIPER_DATA_INVERTED false // đặt true nếu đọc ra giá trị đảo bit
#define LIGHT_PIN        4          // đèn flash trên board AI-Thinker ESP32-CAM

// --- Servo (LEDC) ---
#define SERVO_LEDC_CHANNEL 4        // tránh channel 0/timer0 mà camera XCLK dùng
#define SERVO_FREQ_HZ      50
#define SERVO_RES_BITS     16
#define SERVO_MIN_US       500      // ~0 độ
#define SERVO_MAX_US       2500     // ~180 độ
#define SERVO_STEP_DEG     1        // mỗi bước dịch bao nhiêu độ
#define SERVO_STEP_MS      15       // khoảng thời gian giữa 2 bước (tốc độ)

// --- Chân camera AI-Thinker ESP32-CAM ---
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// ====================== BIẾN TOÀN CỤC =======================================
static httpd_handle_t camera_httpd = NULL;
static httpd_handle_t stream_httpd = NULL;

static volatile float g_caliper_mm  = 0.0f;
static volatile bool  g_caliper_inch = false;
static volatile bool  g_caliper_valid = false;

static int  g_servo_angle  = 10;   // góc hiện tại (đang phát PWM)
static int  g_servo_target = 10;   // góc đích muốn tới
static bool g_light_on     = false;

static Preferences g_prefs;        // lưu SSID/mật khẩu WiFi vào NVS

// Hẹn giờ tắt AP (để điện thoại rời WiFi của ESP, chuyển sang mạng ngoài)
static volatile bool     g_ap_off_pending = false;
static volatile uint32_t g_ap_off_at      = 0;

// Thông tin mạng STA đang dùng (giữ trong RAM để watchdog tự nối lại)
static String g_sta_ssid = "";
static String g_sta_pass = "";

// ====================== THƯỚC KẸP (CALIPER) =================================
// Giao thức phổ biến của thước kẹp TQ giá rẻ: gói 24 bit, LSB trước, lấy mẫu
// data ở cạnh xuống của clock. Giữa 2 gói có khoảng nghỉ dài (~100ms).
// bit 0..15 : độ lớn (đơn vị 0.01mm) | bit 20 : dấu | bit 23 : inch/mm
static volatile uint32_t s_rawBits   = 0;
static volatile uint8_t  s_bitCount  = 0;
static volatile uint32_t s_lastEdgeUs = 0;
static volatile uint32_t s_packetBits = 0;
static volatile bool     s_packetReady = false;

void IRAM_ATTR caliperClkISR() {
  uint32_t now = micros();
  // Khoảng nghỉ dài -> bắt đầu gói mới
  if (now - s_lastEdgeUs > 3000) {
    s_rawBits  = 0;
    s_bitCount = 0;
  }
  s_lastEdgeUs = now;

  bool bit = digitalRead(CALIPER_DATA_PIN);
  if (CALIPER_DATA_INVERTED) bit = !bit;

  if (s_bitCount < 24) {
    if (bit) s_rawBits |= (1UL << s_bitCount);
    s_bitCount++;
    if (s_bitCount == 24) {
      s_packetBits  = s_rawBits;
      s_packetReady = true;
    }
  }
}

// Gọi định kỳ trong loop() để giải mã gói đã nhận đủ
void caliperPoll() {
  if (!s_packetReady) return;
  noInterrupts();
  uint32_t bits = s_packetBits;
  s_packetReady = false;
  interrupts();

  uint32_t magnitude = bits & 0xFFFF;   // bit 0..15
  bool     negative  = bits & (1UL << 20);
  bool     isInch    = bits & (1UL << 23);

  float value;
  if (isInch) {
    value = magnitude / 2000.0f * 25.4f; // inch step = 0.0005" -> mm
  } else {
    value = magnitude / 100.0f;          // mm step = 0.01mm
  }
  if (negative) value = -value;

  g_caliper_mm    = value;
  g_caliper_inch  = isInch;
  g_caliper_valid = true;
}

// ============================ SERVO =========================================
static inline uint32_t usToDuty(int us) {
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  const uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ; // 20000us @50Hz
  return (uint32_t)((uint64_t)us * maxDuty / periodUs);
}

void servoInit() {
  ledcSetup(SERVO_LEDC_CHANNEL, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
}

// Phát PWM ngay tới 1 góc (không mượt) - dùng nội bộ
void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  g_servo_angle = angle;
  int us = map(angle, 0, 180, SERVO_MAX_US, SERVO_MIN_US); // đảo chiều quay
  ledcWrite(SERVO_LEDC_CHANNEL, usToDuty(us));
}

// Đặt góc đích; servo sẽ tự dịch dần tới đó trong servoUpdate()
void servoSetTarget(int angle) {
  g_servo_target = constrain(angle, 0, 180);
}

// Gọi định kỳ trong loop(): dịch góc hiện tại từng bước nhỏ về phía đích
void servoUpdate() {
  static uint32_t lastStepMs = 0;
  if (g_servo_angle == g_servo_target) return;
  uint32_t now = millis();
  if (now - lastStepMs < SERVO_STEP_MS) return;
  lastStepMs = now;

  int diff = g_servo_target - g_servo_angle;
  int step = constrain(diff, -SERVO_STEP_DEG, SERVO_STEP_DEG);
  servoWrite(g_servo_angle + step);
}

// ============================ ĐÈN (LIGHT) ===================================
void lightInit() {
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);
}

void lightWrite(bool on) {
  g_light_on = on;
  digitalWrite(LIGHT_PIN, on ? HIGH : LOW);
}

// ============================ CAMERA ========================================
bool cameraInit() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init that bai: 0x%x\n", err);
    return false;
  }

  // Xoay hiển thị 180°: lật cả dọc (vflip) lẫn ngang (hmirror)
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }
  return true;
}

// ============================ WEB UI ========================================
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="vi"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Calliper</title>
<style>
  :root{
    --bg:#0b1120;--bg2:#111c33;--card:rgba(30,41,59,.55);--line:rgba(148,163,184,.14);
    --txt:#e8eef7;--mut:#93a4bd;--accent:#38bdf8;--accent2:#6366f1;--lime:#a3e635;
  }
  *{box-sizing:border-box}
  body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:0;
       color:var(--txt);text-align:center;-webkit-tap-highlight-color:transparent;
       background:radial-gradient(1200px 600px at 50% -10%,#1e2a4a 0%,var(--bg) 55%) fixed;
       min-height:100vh}
  header{position:sticky;top:0;z-index:5;display:flex;align-items:center;justify-content:center;
         gap:9px;padding:9px 14px;font-size:16px;font-weight:700;letter-spacing:.3px;
         background:rgba(11,17,32,.72);backdrop-filter:blur(10px);
         border-bottom:1px solid var(--line)}
  header .dot{width:9px;height:9px;border-radius:50%;background:var(--lime);
         box-shadow:0 0 0 0 rgba(163,230,53,.6);animation:pulse 2s infinite}
  @keyframes pulse{0%{box-shadow:0 0 0 0 rgba(163,230,53,.55)}
         70%{box-shadow:0 0 0 7px rgba(163,230,53,0)}100%{box-shadow:0 0 0 0 rgba(163,230,53,0)}}
  .brand{background:linear-gradient(90deg,var(--accent),var(--accent2));
         -webkit-background-clip:text;background-clip:text;color:transparent}
  .wrap{max-width:544px;margin:0 auto;padding:10px}
  .frame{position:relative;border-radius:16px;overflow:hidden;background:#000;
         border:1px solid var(--line);box-shadow:0 12px 34px rgba(0,0,0,.45)}
  img{display:block;width:100%;max-width:100%;background:#000}
  .live{position:absolute;top:10px;left:10px;display:flex;align-items:center;gap:6px;
        padding:4px 9px;font-size:11px;font-weight:700;letter-spacing:.6px;border-radius:999px;
        background:rgba(0,0,0,.5);backdrop-filter:blur(4px);color:#fff}
  .live i{width:7px;height:7px;border-radius:50%;background:#f43f5e;display:inline-block;
        animation:pulse 1.6s infinite}
  .card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:11px 14px;
        margin-top:10px;backdrop-filter:blur(8px);box-shadow:0 8px 24px rgba(0,0,0,.28)}
  .label{font-size:12px;font-weight:600;letter-spacing:.8px;text-transform:uppercase;color:var(--mut)}
  .val{font-size:38px;font-weight:800;line-height:1.05;font-variant-numeric:tabular-nums;
       background:linear-gradient(90deg,var(--accent),#818cf8);
       -webkit-background-clip:text;background-clip:text;color:transparent}
  .unit{font-size:15px;color:var(--mut)}
  .row{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:6px}
  .angle{font-size:20px;font-weight:800;color:var(--lime);min-width:52px;
         font-variant-numeric:tabular-nums}
  input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:8px;border-radius:999px;
         background:linear-gradient(90deg,var(--accent),var(--accent2));outline:none;margin:4px 0}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:24px;height:24px;
         border-radius:50%;background:#fff;border:3px solid var(--accent);cursor:pointer;
         box-shadow:0 2px 8px rgba(0,0,0,.4)}
  input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:#fff;
         border:3px solid var(--accent);cursor:pointer}
  .btns{display:flex;gap:10px;margin-top:8px}
  button{flex:1;padding:10px;font-size:15px;font-weight:700;border:1px solid var(--line);
         border-radius:12px;background:rgba(51,65,85,.6);color:var(--txt);cursor:pointer;
         user-select:none;touch-action:none;transition:transform .06s,background .15s,filter .15s}
  button:active{transform:scale(.97);filter:brightness(1.15)}
  button:disabled{opacity:.45;cursor:default}
  .light.on{background:linear-gradient(90deg,#f59e0b,#eab308);color:#1e293b;border-color:transparent}
  .wifi{background:linear-gradient(90deg,var(--accent),var(--accent2));border-color:transparent;color:#051225}
  select,input[type=password]{width:100%;padding:12px;margin-top:10px;border:1px solid var(--line);
         border-radius:12px;background:rgba(11,17,32,.7);color:var(--txt);font-size:15px}
  .status{margin-top:10px;font-size:13px;color:var(--mut);min-height:16px}
  .status.ok{color:var(--lime)}
  .status.err{color:#f87171}
  .overlay{position:fixed;inset:0;background:rgba(2,6,18,.7);backdrop-filter:blur(4px);display:none;
           align-items:center;justify-content:center;z-index:10;padding:16px}
  .overlay.show{display:flex;animation:fade .2s ease}
  @keyframes fade{from{opacity:0}to{opacity:1}}
  .modal{background:linear-gradient(180deg,#16233f,#101a30);border:1px solid var(--line);
         border-radius:18px;padding:20px;width:100%;max-width:360px;text-align:left;position:relative;
         box-shadow:0 20px 60px rgba(0,0,0,.6)}
  .modal h3{margin:0 0 6px;font-size:18px;text-align:center}
  .modal button{width:100%;margin-top:10px}
  .modal .close{position:absolute;top:10px;right:12px;width:auto;margin:0;padding:2px 9px;
                background:none;border:0;font-size:24px;color:var(--mut)}
  .switch{background:linear-gradient(90deg,var(--lime),#65a30d);color:#0b1120;border-color:transparent}
</style></head><body>
<header><span class="dot"></span><span class="brand">ESP32-CAM Calliper</span></header>
<div class="wrap">
  <div class="frame">
    <img id="stream" alt="stream">
    <span class="live"><i></i>LIVE</span>
  </div>

  <div class="card">
    <div class="btns">
      <button id="snap" class="wifi">&#128247; Chụp</button>
      <button id="save" class="switch">&#128190; Lưu ảnh</button>
    </div>
    <div id="snapStat" class="status"></div>
  </div>

  <div class="card">
    <div class="row">
      <span class="label">Servo</span>
      <span id="angle" class="angle">10&deg;</span>
    </div>
    <input id="slider" type="range" min="0" max="180" value="10">
    <div class="btns">
      <button id="back">&#9664; Lùi</button>
      <button id="fwd">Tiến &#9654;</button>
    </div>
  </div>

  <div class="card">
    <div class="btns">
      <button id="light" class="light">&#128161; Bật đèn</button>
      <button id="wifiBtn" class="wifi">&#128246; WiFi</button>
    </div>
  </div>

  <div id="photoCard" class="card" style="display:none">
    <div class="label" style="text-align:left">Ảnh đã chụp</div>
    <img id="photoImg" style="width:100%;border-radius:12px;margin-top:8px" alt="ảnh chụp">
    <div class="btns">
      <button id="photoSave" class="switch">&#128190; Lưu về máy</button>
      <button id="photoHide">&#10005; Ẩn</button>
    </div>
  </div>
</div>

<div id="wifiOverlay" class="overlay">
  <div class="modal">
    <button id="wifiClose" class="close">&times;</button>
    <h3>&#128246; Kết nối WiFi</h3>
    <div id="wifiForm">
      <button id="scanBtn">&#128269; Quét mạng</button>
      <select id="ssidSel"><option value="">-- Chọn mạng --</option></select>
      <input id="passIn" type="password" placeholder="Mật khẩu WiFi">
      <button id="connBtn" class="wifi">&#128279; Kết nối</button>
    </div>
    <div id="wifiDone" style="display:none;text-align:center">
      <div class="unit">Đã kết nối <b id="doneSsid"></b> tại</div>
      <div id="doneIp" style="font-size:22px;font-weight:800;color:var(--lime);word-break:break-all;margin-top:4px"></div>
      <button id="switchBtn" class="switch">&#128241; Chuyển sang IP này</button>
    </div>
    <div id="wifiStat" class="status"></div>
  </div>
</div>

<script>
  // Stream nằm ở cổng 81 -> trỏ img sang đúng host:81
  document.getElementById('stream').src=
    location.protocol+'//'+location.hostname+':81/stream';
  const slider=document.getElementById('slider');
  const angleEl=document.getElementById('angle');
  let t=null;
  function sendAngle(){
    clearTimeout(t);
    t=setTimeout(()=>fetch('/servo?angle='+slider.value),40);
  }
  function setAngle(a){
    a=Math.max(0,Math.min(180,a));
    slider.value=a;
    angleEl.textContent=a+'°';
    sendAngle();
  }
  slider.addEventListener('input',()=>{
    angleEl.textContent=slider.value+'°';
    sendAngle();
  });

  // Khi tải/làm mới trang: đưa servo về góc mặc định 10°
  setAngle(10);

  // Nút Tiến/Lùi: nhấn giữ để chạy liên tục, bỏ ra thì giữ nguyên vị trí
  let hold=null;
  function startHold(step){
    stopHold();
    setAngle(parseInt(slider.value)+step);     // bước đầu ngay khi nhấn
    hold=setInterval(()=>setAngle(parseInt(slider.value)+step),120);
  }
  function stopHold(){ if(hold){clearInterval(hold);hold=null;} }
  function bindHold(id,step){
    const b=document.getElementById(id);
    b.addEventListener('pointerdown',e=>{e.preventDefault();startHold(step);});
    ['pointerup','pointerleave','pointercancel'].forEach(ev=>
      b.addEventListener(ev,stopHold));
  }
  bindHold('fwd', +1);
  bindHold('back',-1);

  // Nút bật/tắt đèn
  const lightBtn=document.getElementById('light');
  let lightOn=false;
  lightBtn.addEventListener('click',async()=>{
    lightOn=!lightOn;
    lightBtn.classList.toggle('on',lightOn);
    lightBtn.innerHTML=lightOn?'&#128161; Tắt đèn':'&#128161; Bật đèn';
    try{await fetch('/light?on='+(lightOn?1:0));}catch(e){}
  });

  // ---- WiFi popup: quét, chọn mạng, kết nối ----
  const wifiBtn=document.getElementById('wifiBtn');
  const overlay=document.getElementById('wifiOverlay');
  const wifiClose=document.getElementById('wifiClose');
  const wifiForm=document.getElementById('wifiForm');
  const wifiDone=document.getElementById('wifiDone');
  const scanBtn=document.getElementById('scanBtn');
  const connBtn=document.getElementById('connBtn');
  const ssidSel=document.getElementById('ssidSel');
  const passIn=document.getElementById('passIn');
  const wifiStat=document.getElementById('wifiStat');
  const doneIp=document.getElementById('doneIp');
  const doneSsid=document.getElementById('doneSsid');
  const switchBtn=document.getElementById('switchBtn');
  let staIp='';

  function openWifi(){ overlay.classList.add('show'); refreshWifi(); }
  function closeWifi(){ overlay.classList.remove('show'); }
  function showForm(){ wifiForm.style.display='block'; wifiDone.style.display='none'; }
  function showDone(ssid,ip){
    staIp=ip;
    doneSsid.textContent=ssid;
    doneIp.textContent=ip;
    wifiForm.style.display='none';
    wifiDone.style.display='block';
    wifiStat.textContent='';
  }

  wifiBtn.addEventListener('click',openWifi);
  wifiClose.addEventListener('click',closeWifi);
  overlay.addEventListener('click',e=>{ if(e.target===overlay) closeWifi(); });

  // Chuyển chế độ: yêu cầu ESP tắt AP, đợi điện thoại tự nối lại mạng nhà,
  // rồi chuyển trang web sang IP mới.
  switchBtn.addEventListener('click',async()=>{
    if(!staIp) return;
    switchBtn.disabled=true;
    wifiStat.className='status';
    wifiStat.textContent='Đang tắt WiFi của ESP, chuyển sang '+staIp+' ...';
    try{ await fetch('/switch'); }catch(e){}
    // ESP tắt AP sau ~1.5s; đợi điện thoại nối lại mạng nhà rồi mới mở IP mới
    setTimeout(()=>{ location.href='http://'+staIp+'/'; }, 4000);
  });

  async function refreshWifi(){
    try{
      const j=await(await fetch('/wifi')).json();
      if(j.connected){ showDone(j.ssid,j.ip); }
      else{
        showForm();
        wifiStat.className='status';
        wifiStat.textContent='Chưa kết nối mạng ngoài';
      }
    }catch(e){ showForm(); }
  }

  scanBtn.addEventListener('click',async()=>{
    scanBtn.disabled=true;
    wifiStat.className='status';
    wifiStat.textContent='Đang quét...';
    try{
      const list=await(await fetch('/scan')).json();
      list.sort((a,b)=>b.rssi-a.rssi);
      ssidSel.innerHTML='<option value="">-- Chọn mạng --</option>';
      for(const n of list){
        const o=document.createElement('option');
        o.value=n.ssid;
        o.textContent=(n.lock?'🔒 ':'📶 ')+n.ssid+'  ('+n.rssi+'dBm)';
        ssidSel.appendChild(o);
      }
      wifiStat.textContent='Tìm thấy '+list.length+' mạng';
    }catch(e){ wifiStat.className='status err'; wifiStat.textContent='Quét lỗi'; }
    scanBtn.disabled=false;
  });

  connBtn.addEventListener('click',async()=>{
    const ssid=ssidSel.value;
    if(!ssid){ wifiStat.className='status err'; wifiStat.textContent='Hãy chọn mạng trước'; return; }
    connBtn.disabled=true;
    wifiStat.className='status';
    wifiStat.textContent='Đang kết nối...';
    try{
      const j=await(await fetch('/connect?ssid='+encodeURIComponent(ssid)+
                                '&pass='+encodeURIComponent(passIn.value))).json();
      if(j.ok){
        wifiStat.className='status ok';
        showDone(ssid,j.ip);
      }else{
        wifiStat.className='status err';
        wifiStat.textContent='Thất bại, kiểm tra lại mật khẩu';
      }
    }catch(e){ wifiStat.className='status err'; wifiStat.textContent='Lỗi kết nối'; }
    connBtn.disabled=false;
  });
  // Chụp / Lưu ảnh: lấy 1 khung hình từ /capture
  const snap=document.getElementById('snap');
  const save=document.getElementById('save');
  const snapStat=document.getElementById('snapStat');
  const photoCard=document.getElementById('photoCard');
  const photoImg=document.getElementById('photoImg');
  const photoHide=document.getElementById('photoHide');
  const photoSave=document.getElementById('photoSave');
  let shotUrl=null;

  function tsName(){
    return 'calliper-'+new Date().toISOString().replace(/[:T]/g,'-').replace(/[.:Z].*$/,'')+'.jpg';
  }
  function download(url){
    const a=document.createElement('a');
    a.href=url; a.download=tsName();
    document.body.appendChild(a); a.click(); a.remove();
  }
  async function capture(){                 // trả về object URL của ảnh vừa chụp
    const r=await fetch('/capture');
    if(!r.ok) throw 0;
    const blob=await r.blob();
    if(shotUrl) URL.revokeObjectURL(shotUrl);
    shotUrl=URL.createObjectURL(blob);
    return shotUrl;
  }
  // Chụp và xem lại ngay dưới cùng trang (giao diện vẫn live, điều khiển bình thường)
  snap.addEventListener('click',async()=>{
    snap.disabled=true; snapStat.className='status'; snapStat.textContent='Đang chụp…';
    try{
      photoImg.src=await capture();
      photoCard.style.display='block';
      photoCard.scrollIntoView({behavior:'smooth',block:'end'});
      snapStat.textContent='';
    }catch(e){ snapStat.className='status err'; snapStat.textContent='Lỗi chụp ảnh'; }
    snap.disabled=false;
  });
  // Chụp và tải thẳng về máy
  save.addEventListener('click',async()=>{
    save.disabled=true; snapStat.className='status'; snapStat.textContent='Đang chụp…';
    try{
      download(await capture());
      snapStat.className='status ok'; snapStat.textContent='Đã lưu ảnh';
    }catch(e){ snapStat.className='status err'; snapStat.textContent='Lỗi chụp ảnh'; }
    save.disabled=false;
  });
  photoHide.addEventListener('click',()=>photoCard.style.display='none');
  photoSave.addEventListener('click',()=>{ if(shotUrl) download(shotUrl); });
</script>
</body></html>
)HTML";

// ====================== TIỆN ÍCH CHUỖI ======================================
// Giải mã URL (%xx và '+') cho SSID / mật khẩu gửi qua query string
static void urlDecode(const char *src, char *dst, size_t dstSize) {
  auto hex = [](char h) -> int {
    if (h >= '0' && h <= '9') return h - '0';
    if (h >= 'a' && h <= 'f') return h - 'a' + 10;
    if (h >= 'A' && h <= 'F') return h - 'A' + 10;
    return 0;
  };
  size_t di = 0;
  for (size_t i = 0; src[i] && di + 1 < dstSize; i++) {
    char c = src[i];
    if (c == '+') {
      c = ' ';
    } else if (c == '%' && src[i + 1] && src[i + 2]) {
      c = (char)(hex(src[i + 1]) * 16 + hex(src[i + 2]));
      i += 2;
    }
    dst[di++] = c;
  }
  dst[di] = 0;
}

// Escape chuỗi để nhét an toàn vào JSON (SSID có thể chứa " \ ...)
static String jsonEscape(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else if (c == '\t') o += "\\t";
    else if ((unsigned char)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
    else o += c;
  }
  return o;
}

// ============================ WIFI STA ======================================
// Thử kết nối tới 1 mạng WiFi. Thành công thì lưu SSID/mật khẩu vào NVS.
bool wifiTryConnect(const char *ssid, const char *pass, uint32_t timeoutMs = 8000) {
  if (!ssid || strlen(ssid) == 0) return false;
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    g_sta_ssid = ssid;   // để watchdog tự nối lại khi rớt
    g_sta_pass = pass;
    g_prefs.begin("wifi", false);
    g_prefs.putString("ssid", ssid);
    g_prefs.putString("pass", pass);
    g_prefs.end();
    Serial.printf("WiFi da ket noi: %s  IP: %s\n", ssid, WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.printf("WiFi ket noi that bai: %s\n", ssid);
  return false;
}

// Nạp SSID/mật khẩu đã lưu và tự kết nối lại khi khởi động
void wifiConnectSaved() {
  g_prefs.begin("wifi", true);
  String ssid = g_prefs.getString("ssid", "");
  String pass = g_prefs.getString("pass", "");
  g_prefs.end();
  if (ssid.length() > 0) {
    Serial.printf("Thu ket noi WiFi da luu: %s\n", ssid.c_str());
    wifiTryConnect(ssid.c_str(), pass.c_str());
  }
}

// ============================ HTTP HANDLERS =================================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t servo_handler(httpd_req_t *req) {
  char buf[64];
  int angle = g_servo_target;
  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen > 1 && qlen <= sizeof(buf)) {
    if (httpd_req_get_url_query_str(req, buf, qlen) == ESP_OK) {
      char val[8];
      if (httpd_query_key_value(buf, "angle", val, sizeof(val)) == ESP_OK) {
        angle = atoi(val);
      }
    }
  }
  servoSetTarget(angle);
  httpd_resp_set_type(req, "application/json");
  char out[48];
  snprintf(out, sizeof(out), "{\"angle\":%d}", g_servo_target);
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t light_handler(httpd_req_t *req) {
  char buf[64];
  bool on = g_light_on;
  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen > 1 && qlen <= sizeof(buf)) {
    if (httpd_req_get_url_query_str(req, buf, qlen) == ESP_OK) {
      char val[8];
      if (httpd_query_key_value(buf, "on", val, sizeof(val)) == ESP_OK) {
        on = atoi(val) != 0;
      }
    }
  }
  lightWrite(on);
  httpd_resp_set_type(req, "application/json");
  char out[32];
  snprintf(out, sizeof(out), "{\"on\":%s}", g_light_on ? "true" : "false");
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

// Quét các mạng WiFi xung quanh -> mảng JSON {ssid, rssi, lock}
static esp_err_t scan_handler(httpd_req_t *req) {
  int n = WiFi.scanNetworks();
  String out = "[";
  for (int i = 0; i < n; i++) {
    if (i) out += ',';
    out += "{\"ssid\":\"";
    out += jsonEscape(WiFi.SSID(i));
    out += "\",\"rssi\":";
    out += WiFi.RSSI(i);
    out += ",\"lock\":";
    out += (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) ? "true" : "false";
    out += '}';
  }
  out += ']';
  WiFi.scanDelete();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out.c_str(), HTTPD_RESP_USE_STRLEN);
}

// Kết nối tới mạng đã chọn; thành công thì lưu mật khẩu
static esp_err_t connect_handler(httpd_req_t *req) {
  char buf[256], ssid[64] = {0}, pass[80] = {0}, tmp[80];
  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen > 1 && qlen <= sizeof(buf) &&
      httpd_req_get_url_query_str(req, buf, qlen) == ESP_OK) {
    if (httpd_query_key_value(buf, "ssid", tmp, sizeof(tmp)) == ESP_OK)
      urlDecode(tmp, ssid, sizeof(ssid));
    if (httpd_query_key_value(buf, "pass", tmp, sizeof(tmp)) == ESP_OK)
      urlDecode(tmp, pass, sizeof(pass));
  }
  bool ok = wifiTryConnect(ssid, pass);
  char out[128];
  if (ok)
    snprintf(out, sizeof(out), "{\"ok\":true,\"ip\":\"%s\"}",
             WiFi.localIP().toString().c_str());
  else
    snprintf(out, sizeof(out), "{\"ok\":false}");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

// Trạng thái kết nối STA hiện tại
static esp_err_t wifi_handler(httpd_req_t *req) {
  bool c = (WiFi.status() == WL_CONNECTED);
  String out = "{\"connected\":";
  out += c ? "true" : "false";
  out += ",\"ssid\":\"";
  out += c ? jsonEscape(WiFi.SSID()) : String("");
  out += "\",\"ip\":\"";
  out += c ? WiFi.localIP().toString() : String("");
  out += "\"}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out.c_str(), HTTPD_RESP_USE_STRLEN);
}

// Chuyển chế độ: hẹn tắt AP sau ~1.5s để trả response kịp, rồi chỉ còn STA.
// Điện thoại mất WiFi của ESP -> tự nối lại mạng nhà -> truy cập được IP mới.
static esp_err_t switch_handler(httpd_req_t *req) {
  bool c = (WiFi.status() == WL_CONNECTED);
  char out[128];
  if (c) {
    g_ap_off_at      = millis() + 1500;
    g_ap_off_pending = true;
    snprintf(out, sizeof(out), "{\"ok\":true,\"ip\":\"%s\"}",
             WiFi.localIP().toString().c_str());
  } else {
    snprintf(out, sizeof(out), "{\"ok\":false}");
  }
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t caliper_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  char out[96];
  snprintf(out, sizeof(out), "{\"valid\":%s,\"value\":%.2f,\"unit\":\"%s\"}",
           g_caliper_valid ? "true" : "false",
           g_caliper_mm,
           g_caliper_inch ? "mm(inch-src)" : "mm");
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

// Chụp 1 khung hình JPEG và trả về để tải xuống
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part_buf[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } else {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
      if (res == ESP_OK) {
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      }
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break; // client ngắt kết nối
  }
  return res;
}

void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 10;

  httpd_uri_t index_uri   = {"/",        HTTP_GET, index_handler,   NULL};
  httpd_uri_t servo_uri   = {"/servo",   HTTP_GET, servo_handler,   NULL};
  httpd_uri_t cal_uri     = {"/caliper", HTTP_GET, caliper_handler, NULL};
  httpd_uri_t capture_uri = {"/capture", HTTP_GET, capture_handler, NULL};
  httpd_uri_t light_uri   = {"/light",   HTTP_GET, light_handler,   NULL};
  httpd_uri_t scan_uri    = {"/scan",    HTTP_GET, scan_handler,    NULL};
  httpd_uri_t connect_uri = {"/connect", HTTP_GET, connect_handler, NULL};
  httpd_uri_t wifi_uri    = {"/wifi",    HTTP_GET, wifi_handler,    NULL};
  httpd_uri_t switch_uri  = {"/switch",  HTTP_GET, switch_handler,  NULL};

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &servo_uri);
    httpd_register_uri_handler(camera_httpd, &cal_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &light_uri);
    httpd_register_uri_handler(camera_httpd, &scan_uri);
    httpd_register_uri_handler(camera_httpd, &connect_uri);
    httpd_register_uri_handler(camera_httpd, &wifi_uri);
    httpd_register_uri_handler(camera_httpd, &switch_uri);
  }

  // Stream chạy ở cổng riêng để không chặn các request điều khiển
  config.server_port += 1;
  config.ctrl_port   += 1;
  httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ============================ SETUP / LOOP ==================================
void setup() {
  // Tắt brownout detector: servo kéo dòng làm sụt áp -> reset giả.
  // Đây chỉ là giải pháp tạm. Cách đúng: cấp nguồn 5V riêng cho servo.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println();

  // Servo
  servoInit();
  servoWrite(10);
  servoSetTarget(10);

  // Đèn
  lightInit();

  // Caliper
  pinMode(CALIPER_CLK_PIN, INPUT);
  pinMode(CALIPER_DATA_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CALIPER_CLK_PIN), caliperClkISR, FALLING);

  // Camera
  if (!cameraInit()) {
    Serial.println("Dung lai vi camera loi.");
  }

  // WiFi: vừa phát AP (để điều khiển) vừa bật STA (để bắt WiFi ngoài)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP SSID: %s  PASS: %s\n", AP_SSID, AP_PASS);
  Serial.printf("Mo trinh duyet: http://%s/\n", ip.toString().c_str());

  // Tự kết nối lại mạng WiFi đã lưu (nếu có)
  wifiConnectSaved();

  startServer();
}

void loop() {
  caliperPoll();
  servoUpdate();

  // Đến hẹn thì tắt riêng interface AP, GIỮ NGUYÊN kết nối STA.
  // (Không đổi WiFi.mode() vì thao tác đó làm STA rớt kết nối.)
  if (g_ap_off_pending && (int32_t)(millis() - g_ap_off_at) >= 0) {
    g_ap_off_pending = false;
    WiFi.enableAP(false);   // chỉ tắt AP, STA vẫn kết nối
    Serial.printf("Da tat AP. STA van ket noi, IP: %s\n",
                  WiFi.localIP().toString().c_str());
  }

  // Watchdog: nếu STA rớt mà đã có mạng đã lưu -> tự kết nối lại
  static uint32_t lastWdt = 0;
  if (g_sta_ssid.length() > 0 && millis() - lastWdt > 5000) {
    lastWdt = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("STA roi ket noi -> thu ket noi lai...");
      WiFi.begin(g_sta_ssid.c_str(), g_sta_pass.c_str());
    }
  }

  delay(2);
}
