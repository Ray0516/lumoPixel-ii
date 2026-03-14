#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <WiFi.h>
#include <WebServer.h>
#include "time.h"

// 引腳定義
#define BUTTON 12
#define FLASH_GPIO 4
#define RED_LED_GPIO 33
#define BUZZER_PIN 4

// 相機引腳 (OV2640 Standard)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// 全域變數
bool cameraReady = false, sdReady = false, firstPhotoTaken = false, lastButtonState = HIGH, apMode = false;
unsigned long buttonPressStart = 0;
const unsigned long MODE_PRESS_TIME = 2000, AP_PRESS_TIME = 5000;
sensor_t *sensor = NULL;
bool isProcessed = false; // 新增旗標，放在 handleButton 外面或作為 static

enum CameraMode { DEFAULT_MODE, PORTRAIT_MODE };
CameraMode currentMode = DEFAULT_MODE;

// 統計資訊結構
struct SystemStats {
    int photoCount;
    uint64_t totalBytes;
    uint64_t usedBytes;
    String lastMode;
} stats;

WebServer server(80);

// --- 工具函式 ---
void handleIcon() {
    // 假設你把圖標放在 SD 卡根目錄，檔名為 icon.png
    // 建議尺寸：192x192 像素
    if (SD_MMC.exists("/icon.png")) {
        File file = SD_MMC.open("/icon.png", FILE_READ);
        server.streamFile(file, "image/png");
        file.close();
    } else {
        server.send(404, "text/plain", "Icon Not Found");
    }
}

void handleJS() {
    // 嘗試不同的路徑格式，增加相容性
    File file = SD_MMC.open("/qrcode.min.js", FILE_READ);
    if (!file) file = SD_MMC.open("qrcode.min.js", FILE_READ); 
    if (!file) file = SD_MMC.open("/sdcard/qrcode.min.js", FILE_READ);

    if (file) {
        server.streamFile(file, "application/javascript");
        file.close();
        Serial.println("成功傳送 JS 檔案");
    } else {
        server.send(404, "text/plain", "JS File Not Found");
        Serial.println("錯誤：SD 卡讀不到 qrcode.min.js");
    }
}

void disableLogs() { esp_log_level_set("*", ESP_LOG_NONE); }

void updateStats() {
    int count = 0;
    File root = SD_MMC.open("/Lumopixel");
    if (root) {
        File file = root.openNextFile();
        while (file) {
            count++;
            file = root.openNextFile();
        }
    }
    stats.photoCount = count;
    stats.totalBytes = SD_MMC.totalBytes();
    stats.usedBytes = SD_MMC.usedBytes();
    stats.lastMode = (currentMode == DEFAULT_MODE) ? "預設模式" : "人像模式";
}

void printStatus() {
    Serial.println("-------------------------");
    Serial.print("Camera: "); Serial.println(cameraReady ? "OK" : "FAIL");
    Serial.print("SD Card: "); Serial.println(sdReady ? "OK" : "FAIL");
    Serial.print("Mode: "); Serial.println(currentMode == DEFAULT_MODE ? "預設" : "人像");
    Serial.println("-------------------------");
}

void flashError() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(FLASH_GPIO, HIGH); delay(150);
        digitalWrite(FLASH_GPIO, LOW); delay(150);
    }
}

// --- 初始化函式 ---

bool initSD() {
    bool ok = SD_MMC.begin("/sdcard", true);
    if (ok) {
        Serial.println("[SD] 初始化完成");
        if (!SD_MMC.exists("/Lumopixel")) SD_MMC.mkdir("/Lumopixel");
    } else Serial.println("[SD] 初始化失敗");
    return ok;
}

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000; 
    config.pixel_format = PIXFORMAT_JPEG; 
    
    // 使用 PSRAM 優化
    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA; // 1600x1200
        config.jpeg_quality = 15;           // 10-12 是最佳平衡點
        config.fb_count = 2;                // 雙緩衝，避免存檔時系統卡死
   }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) { Serial.printf("[Camera] 初始化失敗 0x%x", err); return false; }

    sensor = esp_camera_sensor_get();
    if (sensor) {
        // 核心色彩校正：開啟自動白平衡增益與自動曝光算法
        sensor->set_whitebal(sensor, 1);
        sensor->set_awb_gain(sensor, 1);
        sensor->set_wb_mode(sensor, 0);     // 自動模式
        sensor->set_aec2(sensor, 1);        // 開啟增強型自動曝光控制 (針對室內外切換更準)
        sensor->set_dcw(sensor, 1);         // 開啟數位色彩清理 (減少邊緣雜訊)
        sensor->set_bpc(sensor, 1);         // 修復壞點
        sensor->set_wpc(sensor, 1);         // 修復白點
        
        // 初始預設參數 (穩定為主)
        sensor->set_brightness(sensor, 0); 
        sensor->set_contrast(sensor, 0);
        sensor->set_saturation(sensor, 0);
        sensor->set_special_effect(sensor, 0); // 無特效
    }
    cameraReady = true;
    return true;
}

// --- 功能函式 ---

void toggleMode() {
    if (!sensor) return;
     // --- 這裡新增紅燈閃爍邏輯 ---
    digitalWrite(RED_LED_GPIO, HIGH); // 點亮紅燈
    delay(100);                       // 閃爍長度
    digitalWrite(RED_LED_GPIO, LOW);  // 熄滅紅燈
    // --------------------------
    if (currentMode == DEFAULT_MODE) {
        currentMode = PORTRAIT_MODE;
        Serial.println("[Mode] 切換至人像：高亮、鮮豔、高對比");
        
        sensor->set_brightness(sensor, 1);   // 稍微提亮，讓臉部更清晰
        sensor->set_contrast(sensor, 2);     // 提高對比，讓層次感分明
        sensor->set_saturation(sensor, 2);   // 提高飽和，色彩更討喜
        sensor->set_ae_level(sensor, 1);     // 曝光補償 +1，偏向過曝一點點達成日系感
        
        tone(BUZZER_PIN, 1600, 200); // 高音提醒
    } else {
        currentMode = DEFAULT_MODE;
        Serial.println("[Mode] 回到預設：自然、還原");
        
        sensor->set_brightness(sensor, 1);   // 回歸自然
        sensor->set_contrast(sensor, 1);
        sensor->set_saturation(sensor, 1);
        sensor->set_ae_level(sensor, 1);
        
        tone(BUZZER_PIN, 1600, 200);  // 低音提醒
    }
}

void shutterSound() { tone(BUZZER_PIN, 1500, 30); delay(40); tone(BUZZER_PIN, 1800, 20); }

void takePhotoToSD() {
    if (!cameraReady || !sdReady) { flashError(); return; }
    
    digitalWrite(RED_LED_GPIO, HIGH);
    if (currentMode == PORTRAIT_MODE) { digitalWrite(FLASH_GPIO, HIGH); delay(100); } // 稍微加長閃光預熱
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { 
        Serial.println("擷取失敗"); 
        flashError(); 
        digitalWrite(FLASH_GPIO, LOW); 
        digitalWrite(RED_LED_GPIO, LOW); 
        return; 
    }

    char path[64];
    sprintf(path, "/Lumopixel/IMG_%lu.jpg", millis());
    
    File file = SD_MMC.open(path, FILE_WRITE);
    if (file) {
        size_t written = file.write(fb->buf, fb->len);
        file.flush(); // 【關鍵】強制將緩衝區數據寫入物理磁碟
        file.close();
        
        if (written == fb->len) {
            Serial.printf("儲存成功: %s (%d bytes)\n", path, written);
            shutterSound();
        } else {
            Serial.println("寫入長度不符，檔案可能損壞");
            flashError();
        }
    } else {
        Serial.println("開啟檔案失敗");
        flashError();
    }

    esp_camera_fb_return(fb);
    digitalWrite(FLASH_GPIO, LOW);
    digitalWrite(RED_LED_GPIO, LOW);
}

void handleFileView() {
    if (!server.hasArg("file")) {
        server.send(400, "text/plain", "缺少檔名");
        return;
    }
    String fileName = server.arg("file");
    String path = "/Lumopixel/" + fileName;

    if (SD_MMC.exists(path)) {
        File file = SD_MMC.open(path, FILE_READ);
        // 告訴瀏覽器這是一張 JPEG 圖片
        server.streamFile(file, "image/jpeg");
        file.close();
    } else {
        server.send(404, "text/plain", "找不到檔案");
    }
}

void handleFileDelete() {
    if (!server.hasArg("file")) {
        server.send(400, "text/plain", "缺少參數");
        return;
    }

    String fileName = server.arg("file");
    String path = "/Lumopixel/" + fileName;

    if (SD_MMC.exists(path)) {
        if (SD_MMC.remove(path)) {
            // 修改這裡：讓監控視窗一眼看出是「下載後的自動刪除」
            Serial.print(">>> [SUCCESS] 手機已領取並移除檔案: ");
            Serial.println(fileName);
            
            updateStats(); // 記得呼叫這行，讓空間計算正確
            server.send(200, "text/plain", "Deleted Successfully");
        } else {
            Serial.println("[Error] 檔案佔用中或刪除失敗");
            server.send(500, "text/plain", "Delete Failed");
        }
    } else {
        server.send(404, "text/plain", "File Not Found");
    }
}

void handleDownload() {
    if (!server.hasArg("file")) return;
    String path = "/Lumopixel/" + server.arg("file");
    
    if (SD_MMC.exists(path)) {
        File file = SD_MMC.open(path, FILE_READ);
        // 強制瀏覽器跳出下載視窗
        server.sendHeader("Content-Disposition", "attachment; filename=" + server.arg("file"));
        server.streamFile(file, "image/jpeg");
        file.close();
    }
}

// --- 修改後的 handleRoot 函數 ---
void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset='utf-8'>
    <meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover'>
    
    <meta name="apple-mobile-web-app-capable" content="yes">
    <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
    <meta name="mobile-web-app-capable" content="yes">
    <meta name="theme-color" content="#000000">

    <link rel="apple-touch-icon" href="/icon.png">
    <link rel="icon" type="image/png" href="/icon.png">

    <title>LUMOPIXEL GO</title>
    <script src="/qrcode.min.js"></script>
    <style>
        :root { --bg: #000000; --accent: #ffdd00; --card: #111111; --text: #ffffff; }
        * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
        
        body { 
            background: var(--bg); 
            color: var(--text); 
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; 
            margin:0; 
            overflow-x: hidden;
            padding-top: env(safe-area-inset-top); 
        }

        .brand-header { padding: 30px 25px 10px 25px; display: flex; align-items: baseline; gap: 8px; }
        .brand-logo { font-size: 24px; font-weight: 900; letter-spacing: -1.5px; color: var(--text); }
        .brand-go { font-size: 18px; font-weight: 200; opacity: 0.6; }

        .storage-section { padding: 10px 25px 25px 25px; border-bottom: 1px solid #1a1a1a; }
        .storage-label { font-size: 10px; text-transform: uppercase; letter-spacing: 2px; color: #555; margin-bottom: 8px; }
        .storage-wireframe { display: flex; gap: 4px; height: 12px; align-items: center; }
        .segment { flex: 1; height: 2px; background: #1a1a1a; border-radius: 1px; transition: 0.6s cubic-bezier(0.23, 1, 0.32, 1); }
        .segment.active { background: var(--accent); box-shadow: 0 0 8px var(--accent); }

        .gacha-viewport { display: flex; overflow-x: auto; scroll-snap-type: x mandatory; padding: 40px 0; gap: 20px; scrollbar-width: none; }
        .gacha-viewport::-webkit-scrollbar { display: none; }
        .gacha-viewport::before, .gacha-viewport::after { content: ''; flex: 0 0 10%; }
        
        .card { 
            flex: 0 0 80%; aspect-ratio: 3/4; background: #080808; border-radius: 30px; 
            scroll-snap-align: center; overflow: hidden; position: relative; 
            border: 1px solid #222; box-shadow: 0 30px 60px rgba(0,0,0,0.5);
        }

        .card canvas, .card img { 
            width: 100%; 
            height: 100%; 
            object-fit: cover; 
            background: #000;
            display: block;
        }

        .card canvas {
            image-rendering: pixelated;
            display: none; /* 預設隱藏，開啟 PIXEL 模式才顯示 */
        }
        
        .card-info { 
            position: absolute; bottom: 0; left: 0; right: 0; padding: 20px; 
            background: linear-gradient(to top, rgba(0,0,0,0.95) 0%, rgba(0,0,0,0.4) 50%, transparent 100%); 
        }
        .file-name { font-size: 11px; font-family: monospace; color: #999; margin-bottom: 15px; text-align: center; }
        
        .btn-group { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
        
        .btn { 
            border: none; padding: 12px 5px; border-radius: 12px; font-size: 10px; 
            font-weight: 700; cursor: pointer; transition: 0.2s; text-align: center;
        }
        
        .btn-main { background: var(--accent); color: #000; }
        .btn-sub { background: #222; color: #fff; }
        .btn-pixel-active { background: #fff !important; color: #000 !important; }
        .btn-danger { background: rgba(255, 68, 68, 0.1); color: #ff4444; border: 1px solid rgba(255, 68, 68, 0.2); }

        .btn-all { 
            background: #ffffff; color: #000000; width: calc(100% - 50px); 
            margin: 0 25px 50px 25px; display: block; font-size: 12px; padding: 15px; border-radius: 20px;
        }

        #qrOverlay { 
            display:none; position:fixed; top:0; left:0; width:100%; height:100%; 
            background:rgba(0,0,0,0.98); z-index:1000; flex-direction:column; 
            align-items:center; justify-content:center; backdrop-filter: blur(10px);
        }
        #qrcode { background:#fff; padding:20px; border-radius:25px; }
        #qrcode img { width: 220px; height: 220px; }
        .qr-tip { color: var(--accent); margin-top: 25px; font-size: 14px; text-align: center; }
        .empty-state { width: 100%; padding: 100px 0; color: #333; font-weight: 800; text-align:center; }
    </style>
</head>
<body>
    <div class="brand-header">
        <span class="brand-logo">LUMO PIXEL</span><span class="brand-go">GO</span>
    </div>

    <div class="storage-section">
        <div class="storage-label">Storage / <span id="storageText">Checking...</span></div>
        <div id="wireframe" class="storage-wireframe"></div>
    </div>

    <div id="gachaWrapper" class="gacha-viewport">
        <div class="empty-state">LOADING MEMORIES...</div>
    </div>

    <button id="downloadAllBtn" class="btn btn-all" onclick="downloadAll()">DOWNLOAD ALL MEMORIES</button>

    <div id="qrOverlay" onclick="this.style.display='none'">
        <div id="qrcode"></div>
        <div class="qr-tip">SCAN TO DOWNLOAD</div>
        <button class="btn btn-sub" style="margin-top:40px; width:120px;">CLOSE</button>
    </div>

    <script>
        async function loadGacha() {
            try {
                const statusRes = await fetch('/status');
                const status = await statusRes.json();
                document.getElementById('storageText').innerText = status.storage;
                
                const wf = document.getElementById('wireframe');
                wf.innerHTML = '';
                const activeCount = Math.round((status.usage / 100) * 30);
                for(let i=0; i<30; i++) {
                    let s = document.createElement('div');
                    s.className = 'segment' + (i < activeCount ? ' active' : '');
                    wf.appendChild(s);
                }

                const listRes = await fetch('/list');
                const files = await listRes.json();
                const wrapper = document.getElementById('gachaWrapper');
                
                if(!files || files.length === 0) {
                    wrapper.innerHTML = '<div class="empty-state">NO MEMORIES YET</div>';
                    document.getElementById('downloadAllBtn').style.display = 'none';
                    return;
                }

                document.getElementById('downloadAllBtn').style.display = 'block';
                wrapper.innerHTML = '';
                files.reverse().forEach(f => {
                    const card = document.createElement('div');
                    card.className = 'card';
                    card.dataset.filename = f;
                    card.dataset.pixel = "false";
                    card.innerHTML = `
                        <img src="/view?file=${encodeURIComponent(f)}" crossorigin="anonymous" onload="initCanvas(this)">
                        <canvas></canvas>
                        <div class="card-info">
                            <div class="file-name">${f}</div>
                            <div class="btn-group">
                                <button class="btn btn-sub" onclick="togglePixel(this)">PIXEL</button>
                                <button class="btn btn-main" onclick="showQR('${f}', this)">SHARE</button>
                                <button class="btn btn-sub" onclick="downloadImg('${f}', this)">SAVE</button>
                                <button class="btn btn-danger" onclick="deleteImg('${f}')">DEL</button>
                            </div>
                        </div>
                    `;
                    wrapper.appendChild(card);
                });
            } catch (e) { console.error("Load Error:", e); }
        }

        // 初始化 Canvas，將圖片繪製進去但不顯示像素化
        function initCanvas(img) {
            const canvas = img.nextElementSibling;
            canvas.width = img.naturalWidth;
            canvas.height = img.naturalHeight;
            const ctx = canvas.getContext('2d');
            ctx.drawImage(img, 0, 0);
        }

        // 切換像素模式
        function togglePixel(btn) {
            const card = btn.closest('.card');
            const img = card.querySelector('img');
            const canvas = card.querySelector('canvas');
            const isPixel = card.dataset.pixel === "true";

            if (!isPixel) {
                // 開啟像素模式
                const ctx = canvas.getContext('2d');
                const w = canvas.width;
                const h = canvas.height;
                const pixelSize = 10; // 像素顆粒大小，可調整

                // 核心演算法：縮小再放大
                ctx.imageSmoothingEnabled = false;
                ctx.drawImage(img, 0, 0, w/pixelSize, h/pixelSize);
                ctx.drawImage(canvas, 0, 0, w/pixelSize, h/pixelSize, 0, 0, w, h);
                
                canvas.style.display = 'block';
                img.style.display = 'none';
                btn.classList.add('btn-pixel-active');
                card.dataset.pixel = "true";
            } else {
                // 關閉像素模式
                canvas.style.display = 'none';
                img.style.display = 'block';
                btn.classList.remove('btn-pixel-active');
                card.dataset.pixel = "false";
            }
        }

        function getImgSource(fileName, btn) {
            const card = btn.closest('.card');
            if (card.dataset.pixel === "true") {
                return card.querySelector('canvas').toDataURL("image/jpeg", 0.9);
            }
            return '/view?file=' + encodeURIComponent(fileName);
        }

        function showQR(fileName, btn) {
            // 注意：QR Code 只能分享 URL，無法直接分享 Base64 圖片數據。
            // 若要分享像素後的圖，通常是透過「SAVE」後手動發送，
            // 這裡的 SHARE 仍指向原始連結，或提示使用者先儲存。
            const url = `http://192.168.4.1/view?file=${encodeURIComponent(fileName)}`;
            const container = document.getElementById("qrcode");
            container.innerHTML = "";
            new QRCode(container, { text: url, width: 220, height: 220 });
            document.getElementById('qrOverlay').style.display = 'flex';
        }

        function downloadImg(f, btn) {
            const source = getImgSource(f, btn);
            const a = document.createElement('a');
            a.href = source;
            a.download = "pixel_" + f;
            a.click();
        }
        

        async function downloadAll() {
            const cards = document.querySelectorAll('.card');
            if(cards.length === 0) return;
            if(confirm(`下載並清空這 ${cards.length} 張照片？`)) {
                for (const card of cards) {
                    const f = card.dataset.filename;
                    const btn = card.querySelector('.btn-main'); // 借用按鈕定位
                    downloadImg(f, btn);
                    await new Promise(r => setTimeout(r, 600)); 
                    await fetch('/delete?file=' + encodeURIComponent(f));
                }
                setTimeout(() => { loadGacha(); }, 1000);
            }
        }

        async function deleteImg(f) {
            if(confirm('Delete this memory?')) {
                await fetch('/delete?file=' + encodeURIComponent(f));
                loadGacha();
            }
        }
        window.onload = loadGacha;
    </script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}




void handleStatus() {
    // 重新計算目前的空間狀況
    uint64_t total = SD_MMC.totalBytes();
    uint64_t used = SD_MMC.usedBytes();
    int usagePercent = (total > 0) ? (int)((used * 100) / total) : 0;

    String json = "{";
    json += "\"photoCount\":" + String(stats.photoCount) + ",";
    json += "\"usage\":" + String(usagePercent) + ","; // 補上這行，網頁才不會報錯
    json += "\"lastMode\":\"" + stats.lastMode + "\",";
    json += "\"storage\":\"" + String((float)(total - used) / 1024 / 1024, 1) + " MB FREE\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleList() {
    File root = SD_MMC.open("/Lumopixel");
    if (!root || !root.isDirectory()) {
        server.send(200, "application/json", "[]");
        return;
    }

    String json = "[";
    File file = root.openNextFile();
    while (file) {
        if (json != "[") json += ",";
        String name = String(file.name());
        // 如果檔名開頭有 /，把它去掉，避免後端拼接路徑出錯
        if (name.startsWith("/")) name = name.substring(1);
        json += "\"" + name + "\"";
        file = root.openNextFile();
    }
    json += "]";
    server.send(200, "application/json", json);
}

void startAP() {
  Serial.println("[AP] 啟動中...");
     if (sensor) {
        // 不要關掉相機，只是把解析度調到最小，釋放記憶體給網頁用
        sensor->set_framesize(sensor, FRAMESIZE_CIF); 
    }
    // 強力提示：閃爍 3 次快燈 + 嗶嗶聲
    for (int i = 0; i < 3; i++) {
        digitalWrite(RED_LED_GPIO, HIGH);
        tone(BUZZER_PIN, 1000, 80); 
        delay(50);
        digitalWrite(RED_LED_GPIO, LOW);
        delay(50);
    }
    WiFi.softAP("Lumopixel_AP");
    server.on("/qrcode.min.js", handleJS); // 確保這行有加！
    // 啟動前先統計資料
    updateStats();
    
    for (int i = 0; i < 5; i++) { digitalWrite(RED_LED_GPIO, HIGH); delay(200); digitalWrite(RED_LED_GPIO, LOW); delay(200); }
    
    if (cameraReady) {
        esp_camera_deinit();
        cameraReady = false;
        sensor = NULL;
    }
    WiFi.softAP("Lumopixel_AP");
    server.on("/", handleRoot);
    server.on("/list", handleList);
    server.on("/view", handleFileView);
    server.on("/status", handleStatus);
    server.on("/delete", handleFileDelete);
    server.on("/qrcode.min.js", handleJS);
    server.on("/icon.png", handleIcon);
    server.begin();
    apMode = true;
    Serial.println("[AP] 運作中: 192.168.4.1");
}

// --- 主邏輯 ---

void handleButton() {
    bool state = digitalRead(BUTTON);
    unsigned long now = millis();
    static bool apTriggered = false; // 專門紀錄是否已經觸發了 5 秒的超長按

    if (state == LOW && lastButtonState == HIGH) {
        buttonPressStart = now;
        apTriggered = false; // 按下時重置
    }

    // 計算當前按住的時間
    unsigned long pressDuration = (state == LOW) ? (now - buttonPressStart) : 0;

    // --- 實時觸發 (超長按 5 秒) ---
    // 只有 5 秒我們讓它按著就觸發，因為這是重大操作
    if (state == LOW && !apTriggered && pressDuration >= AP_PRESS_TIME) {
        apTriggered = true; 
        startAP(); // 進入 AP 模式（裡面有急促嗶聲與閃燈）
    }

    // --- 放開按鈕時判定 (拍照 或 切換模式) ---
    if (state == HIGH && lastButtonState == LOW) {
        unsigned long finalDuration = now - buttonPressStart;

        if (apTriggered) {
            // 如果剛才已經觸發過 5 秒 AP 模式了，放開後什麼都不做
            Serial.println("[Button] AP模式已觸發，放開不執行動作");
        } 
        else if (finalDuration >= MODE_PRESS_TIME) {
            // 按住時間在 2 ~ 5 秒之間，放開時切換模式
            toggleMode(); 
        } 
        else if (finalDuration > 50) {
            // 按住時間小於 2 秒，執行拍照
            takePhotoToSD();
        }
        
        apTriggered = false; // 放開後重置標記
    }

    lastButtonState = state;
}

// 1. 開機音效：調低音頻，縮短時間
void playStartupSound() {
    tone(BUZZER_PIN, 1300, 80); delay(100); // 低音 Do
    tone(BUZZER_PIN, 1400, 80); delay(100); // 低音 Do
    tone(BUZZER_PIN, 1500, 80); delay(100); // 低音 Mi
    noTone(BUZZER_PIN);
}


// 2. 正確的 setup 寫法
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON, INPUT_PULLUP);
    pinMode(FLASH_GPIO, OUTPUT);
    pinMode(RED_LED_GPIO, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    digitalWrite(FLASH_GPIO, LOW);
    digitalWrite(RED_LED_GPIO, LOW);

    disableLogs();
    sdReady = initSD();
    cameraReady = initCamera();

    if(cameraReady) {
        Serial.println("[System] 執行開機自動校準拍攝...");
        // 連拍兩張廢片效果更好（確保感光元件完全適應光線）
        for(int i=0; i<2; i++){
            camera_fb_t *fb = esp_camera_fb_get();
            if(fb) esp_camera_fb_return(fb);
            delay(100);
        }
        firstPhotoTaken = true; 
        Serial.println("[System] 校準完成");
    }

    playStartupSound(); // 呼叫音效
    printStatus();
    Serial.println("ESP32 啟動完成，等待按鈕操作...");
}

void loop() {
    handleButton();
    if (apMode) server.handleClient();
}