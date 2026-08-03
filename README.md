# lumoPixel-ii / LUMO YI

這個儲存庫收錄兩個相機相關的開源製作專案：

1. **lumoPixel-ii**：以 ESP32-CAM 製作的開源數位相機，包含韌體、3D 列印檔、APK 與完整製作說明。
2. **LUMO YI**：使用 Flutter 與 YI Open API 製作的 YI 4K Action Camera Android 遙控 App，支援 Android 16。

## lumoPixel-ii

原始製作資料保留在儲存庫根目錄：

- `lumopixelv1.ino`：ESP32-CAM Arduino 韌體
- `製作說明_compressed.pdf`：完整製作說明
- `lumopixel_compressed (1).pdf`：補充文件
- `取景器套件.3mf`、`無閃.3mf`、`補光.3mf`：3D 列印模型
- `app-arm64-v8a-release.apk`：原始 lumoPixel Android App

## LUMO YI

程式位於 [`LUMO-YI-Android/`](LUMO-YI-Android/)。這是針對 **YI 4K Action Camera** 製作的手機遙控與媒體管理 App。

### 主要功能

- Android 16／HyperOS 支援
- 相機 Wi-Fi 連線與斷線自動重連
- RTSP 低延遲取景預覽與停滯自動恢復
- 拍照、開始／停止錄影及拍攝模式切換
- 最高 12MP 拍照設定
- 原圖或中央 2× 裁切下載
- 正片、負片、鮮豔、徠卡風格等 10 種色調
- 可調整對比、飽和、色溫、褪色與暗角的自訂濾鏡
- 手機端降噪、局部對比、銳化及高品質 JPEG 處理
- 照片、MP4 與其他相機檔案分類
- 快速縮圖、長按多選、單張或整批下載

### 直接安裝

下載 [`LUMO-YI-Android16-release.apk`](LUMO-YI-Android/release/LUMO-YI-Android16-release.apk)，允許 Android 安裝未知來源 App 後即可安裝。

### 使用方式

1. 在 YI 4K 相機上開啟 Wi-Fi。
2. 手機連接相機建立的 `YDXJ_...` 無線網路。
3. 開啟 LUMO YI，按下「連接相機」。
4. 顯示「沒有網際網路」是正常狀態，手機仍可透過區域 Wi-Fi 控制相機。

### 開發與編譯

需求：Flutter 3、Android Studio、Android SDK 及 JDK 17。

```sh
cd LUMO-YI-Android
flutter pub get
flutter build apk --release
```

相機控制橋接程式位於 `android/app/src/main/kotlin/`，Flutter UI 與操作流程位於 `lib/main.dart`。

## 注意事項

- LUMO YI 是獨立開發的非官方 App，並非 YI Technology、Xiaomi 或 Leica 官方產品。
- 「徠卡風格」僅代表自行調校的色彩風格，不包含或複製任何專有演算法。
- App 使用 YI Technology 公開的 YI Open API 與相機區域網路功能。
- 相機韌體、記憶卡速度及 Wi-Fi 環境會影響預覽延遲與下載速度。

## 參考資料

- [YI Open API](https://github.com/YITechnology/YIOpenAPI)
- [YI 4K Action Camera 規格](https://api.yitechnology.com/yi-4k-action-camera-specs)
