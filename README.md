# Lumo Pixel ii

基於 ESP32-CAM 的開源相機，搭配 LumoPixel GO Android App 讀取、預覽與儲存照片。

## Android App 1.1.1

從本專案的 [GitHub Releases](https://github.com/Ray0516/lumoPixel-ii/releases) 下載最新版 APK。`LumoPixel-GO-1.1.1.apk` 支援 arm64-v8a、armeabi-v7a 與 x86_64，最低支援 Android 7.0。

新版改用淺色介面，移除裝飾標語；照片縮圖保留完整構圖，提供全螢幕縮放、左右切換、多選下載、下載進度、失敗重試與可儲存的濾鏡參數。原圖下載保留相機檔案，不重新壓縮；下載不會自動刪除相機中的照片。

安裝後，長按相機按鈕約 5 秒進入 Wi-Fi 模式，手機連上 `Lumopixel_AP`，再開啟 App。此 Wi-Fi 用於相機傳輸，沒有網際網路時仍需保持連線。QR Code 分享指向相機原圖，接收端也需連上相機 Wi-Fi。

### 濾鏡參數

| 參數 | 範圍 | 預設值 |
| --- | --- | --- |
| 像素密度（縮小後寬度） | 24–240 | 120 |
| 黑白對比 | 0.5–2.0 | 1.1 |
| 黑白亮度 | -50–50 | 0 |
| 復古強度 | 0–1 | 0.65 |
| 濾鏡 JPEG 品質 | 75 / 85 / 95 | 95 |

### 更新相容性與驗證

App 保留 `com.example.flutter_application_1` 識別碼；1.1.1 的 Android versionCode 為 3。發佈 APK 使用與本專案舊 APK 相同的既有簽章，可直接覆蓋更新。

本機已通過靜態分析、12 項自動測試與 Android release 建置。相機 API 的路徑格式已依 `lumopixelv1.ino` 對齊；尚未完成實體手機及相機的端到端驗證。

## 原始碼與建置

Flutter 原始碼位於 [`app/`](app/)，以 Flutter 3.41.4 / Dart 3.11.1 驗證，包含 Android 專案與 Web 預覽入口。

```sh
cd app
flutter pub get
flutter analyze
flutter test
flutter build apk --release
```

本專案的既有 Android 設定使用本機 debug keystore。其他電腦建置會產生不同簽章，無法直接覆蓋目前發佈的 APK；要發佈可覆蓋更新的版本，需使用原簽章。私鑰不放在儲存庫中。

GitHub Actions 自動執行靜態分析及測試。Web 可用於 UI 預覽；存入手機相簿功能需在 Android App 操作。

### 相機通訊

App 連線至 `http://192.168.4.1`，沿用 `/status`、`/list`、`/view?file=...` 與 `/delete?file=...`。檔名以 URI query parameters 編碼；`/list` 回傳完整 SD 路徑或單純檔名時皆轉為檔名，避免重複拼接 `/Lumopixel/`。本次 App 更新不需要重燒相機韌體。

## 相機與外殼檔案

- `lumopixelv1.ino`：既有 ESP32-CAM 韌體。
- `app-arm64-v8a-release.apk`：保留既有 Android 安裝檔，最新版請至 Releases。
- PDF：原相機介紹及製作說明。
- 3MF：取景器、無閃與補光版本外殼。
