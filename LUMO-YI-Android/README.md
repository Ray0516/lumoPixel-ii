# LUMO YI Android

以 Flutter 與 YI Open API 製作的 YI 4K Action Camera Android 16 遙控 App。

## 功能

- 低延遲 RTSP 預覽與停滯自動恢復
- 拍照、錄影、模式切換與相機設定
- 12MP 拍照、手機端高畫質處理及原圖／2× 裁切
- 10 種預設色調與可儲存的自訂濾鏡
- 照片、MP4、其他檔案分類
- 快速縮圖、長按多選、單張／全部下載
- Android 16 相簿整合與 Wi-Fi 自動重連

## 安裝

已編譯版本位於 [`release/LUMO-YI-Android16-release.apk`](release/LUMO-YI-Android16-release.apk)。

## 編譯

```sh
flutter pub get
flutter build apk --release
```

主程式：`lib/main.dart`  
Android 相機橋接：`android/app/src/main/kotlin/com/codexstudio/yi_4k_remote/MainActivity.kt`

本專案為非官方相機控制 App；YI、Xiaomi 與 Leica 名稱及商標屬各自權利人所有。
