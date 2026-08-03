import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:video_player/video_player.dart';

const _accent = Color(0xFFE8C8A0);
const _background = Color(0xFF0B0B0B);
const _surface = Color(0xFF171717);
const _surfaceHigh = Color(0xFF222222);

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const YiRemoteApp());
}

class YiRemoteApp extends StatelessWidget {
  const YiRemoteApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'LUMO YI',
      theme: ThemeData(
        brightness: Brightness.dark,
        useMaterial3: true,
        scaffoldBackgroundColor: _background,
        colorScheme: ColorScheme.fromSeed(
          seedColor: _accent,
          brightness: Brightness.dark,
          surface: _surface,
        ),
        cardTheme: const CardThemeData(
          elevation: 0,
          color: _surface,
          margin: EdgeInsets.zero,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.all(Radius.circular(6)),
          ),
        ),
        navigationBarTheme: const NavigationBarThemeData(
          backgroundColor: Color(0xFF111317),
          indicatorColor: Color(0x22E8C8A0),
          height: 66,
        ),
        inputDecorationTheme: InputDecorationTheme(
          filled: true,
          fillColor: _surfaceHigh,
          border: OutlineInputBorder(
            borderSide: BorderSide.none,
            borderRadius: BorderRadius.circular(14),
          ),
        ),
      ),
      home: const RemoteShell(),
    );
  }
}

class YiCameraController extends ChangeNotifier {
  static const _methods = MethodChannel('yi4k.remote/control');
  static const _events = EventChannel('yi4k.remote/events');

  StreamSubscription<Object?>? _subscription;
  VideoPlayerController? preview;
  bool connected = false;
  bool connecting = false;
  bool recording = false;
  bool photoMode = false;
  bool previewLoading = false;
  bool _restartingPreview = false;
  int? battery;
  String? error;
  String? notice;
  Map<String, Object?> settings = {};
  List<Map<String, Object?>> files = [];
  final Set<String> downloading = {};
  final Set<String> downloaded = {};
  final Map<String, double> downloadProgress = {};
  final Map<String, Future<Uint8List?>> _thumbnailRequests = {};
  bool downloadingAll = false;
  int bulkCompleted = 0;
  int bulkTotal = 0;
  bool crop2x = true;
  bool previewCrop2x = true;
  bool enhancePhotos = true;
  bool autoDownload = true;
  String filmFilter = 'leicaAuthentic';
  double customContrast = 1;
  double customSaturation = 1;
  double customWarmth = 0;
  double customFade = 0;
  double customVignette = .1;
  String galleryCategory = 'photos';
  final Set<String> selectedFiles = {};
  bool _pendingAutoDownload = false;
  bool _handlingCapture = false;
  bool _manualDisconnect = false;
  bool _everConnected = false;
  int _reconnectAttempt = 0;
  Timer? _reconnectTimer;
  Timer? _previewWatchdog;
  Duration _lastPreviewPosition = Duration.zero;
  int _previewStalls = 0;

  YiCameraController() {
    _subscription = _events.receiveBroadcastStream().listen(
      _onEvent,
      onError: _setError,
    );
    _loadPreferences();
  }

  void _onEvent(Object? raw) {
    if (raw is! Map) return;
    final event = Map<String, Object?>.from(raw);
    switch (event['type']) {
      case 'connection':
        final wasConnected = connected;
        connected = event['connected'] == true;
        connecting = false;
        if (connected) {
          _everConnected = true;
          _reconnectAttempt = 0;
          _reconnectTimer?.cancel();
        } else {
          recording = false;
          _disposePreview();
          if (wasConnected) notice = '相機連線中斷，正在嘗試重新連線';
          if (_everConnected && !_manualDisconnect) _scheduleReconnect();
        }
      case 'recording':
        recording = event['active'] == true;
      case 'capture':
        if (event['active'] == false && _pendingAutoDownload) {
          _finishCapturedPhoto();
        }
      case 'viewfinder':
        final url = event['url'] as String?;
        if (event['active'] == true && url != null) {
          _startPreview(url);
        }
      case 'battery':
        battery = event['percent'] as int?;
      case 'settings':
        event.remove('type');
        settings = {...settings, ...event};
        photoMode = settings['systemMode'] == 'Capture';
      case 'download':
        final name = event['name']?.toString();
        if (name != null) {
          if (event['active'] == true) {
            downloading.add(name);
            final bytes = (event['bytes'] as num?)?.toDouble() ?? 0;
            final total = (event['total'] as num?)?.toDouble() ?? 0;
            if (total > 0) downloadProgress[name] = (bytes / total).clamp(0, 1);
          } else {
            downloading.remove(name);
            downloadProgress.remove(name);
            if (event['complete'] == true) downloaded.add(name);
          }
        }
      case 'error':
        _setError(event['message']);
    }
    notifyListeners();
  }

  Future<void> openWifiSettings() => _methods.invokeMethod('openWifiSettings');

  Future<void> _loadPreferences() async {
    try {
      final values = await _methods.invokeMapMethod<String, Object?>(
        'getPreferences',
      );
      if (values == null) return;
      crop2x = values['crop2x'] as bool? ?? crop2x;
      previewCrop2x = values['previewCrop2x'] as bool? ?? previewCrop2x;
      enhancePhotos = values['enhance'] as bool? ?? enhancePhotos;
      autoDownload = values['autoDownload'] as bool? ?? autoDownload;
      filmFilter = values['filter']?.toString() ?? filmFilter;
      if (!_filmFilters.containsKey(filmFilter)) filmFilter = 'negative';
      customContrast = (values['customContrast'] as num?)?.toDouble() ?? 1;
      customSaturation = (values['customSaturation'] as num?)?.toDouble() ?? 1;
      customWarmth = (values['customWarmth'] as num?)?.toDouble() ?? 0;
      customFade = (values['customFade'] as num?)?.toDouble() ?? 0;
      customVignette = (values['customVignette'] as num?)?.toDouble() ?? .1;
      notifyListeners();
    } catch (_) {}
  }

  Future<void> setLocalPreference(String key, Object value) async {
    switch (key) {
      case 'crop2x':
        crop2x = value as bool;
      case 'previewCrop2x':
        previewCrop2x = value as bool;
      case 'enhance':
        enhancePhotos = value as bool;
      case 'autoDownload':
        autoDownload = value as bool;
      case 'filter':
        filmFilter = value.toString();
      case 'customContrast':
        customContrast = (value as num).toDouble();
      case 'customSaturation':
        customSaturation = (value as num).toDouble();
      case 'customWarmth':
        customWarmth = (value as num).toDouble();
      case 'customFade':
        customFade = (value as num).toDouble();
      case 'customVignette':
        customVignette = (value as num).toDouble();
    }
    notifyListeners();
    await _methods.invokeMethod('setPreference', {'key': key, 'value': value});
  }

  Future<void> connect({bool automatic = false}) async {
    if (connecting || connected) return;
    _manualDisconnect = false;
    connecting = true;
    error = null;
    notice = automatic ? '正在重新連接相機…' : null;
    notifyListeners();
    try {
      await _methods.invokeMethod('connect', {'ip': '192.168.42.1'});
      connected = true;
      await refresh();
      await updateSetting(
        method: 'setPhotoResolution',
        key: 'photoResolution',
        value: 'p_12MP_4000x3000_4x3_w',
        message: '拍照已設為最高 12MP',
      );
      if (settings['videoResolution'] == 'v_3840x2160_30p_16x9') {
        await updateSetting(
          method: 'setVideoResolution',
          key: 'videoResolution',
          value: 'v_1920x1080_30p_16x9',
          message: '已改用 1080p／30 fps，降低預覽負擔',
        );
      }
      await startViewfinder();
      if (automatic) notice = '已重新連接相機';
    } catch (exception) {
      _setError(exception);
    }
    connecting = false;
    notifyListeners();
  }

  Future<void> disconnect() async {
    _manualDisconnect = true;
    _reconnectTimer?.cancel();
    await _call('disconnect');
    connected = false;
    await _disposePreview();
    notifyListeners();
  }

  void _scheduleReconnect() {
    if (_reconnectTimer?.isActive == true || _reconnectAttempt >= 3) {
      if (_reconnectAttempt >= 3) {
        notice = '自動重連未成功，請確認手機仍連著相機 Wi‑Fi';
        notifyListeners();
      }
      return;
    }
    final delays = [2, 5, 10];
    final delay = delays[_reconnectAttempt++];
    _reconnectTimer = Timer(Duration(seconds: delay), () {
      if (!connected && !_manualDisconnect) connect(automatic: true);
    });
  }

  Future<void> refresh() async {
    try {
      final raw = await _methods.invokeMapMethod<String, Object?>(
        'getSettings',
      );
      if (raw != null) settings = raw;
      battery = await _methods.invokeMethod<int>('getBattery');
      photoMode = settings['systemMode'] == 'Capture';
      error = null;
    } catch (exception) {
      _setError(exception);
    }
    notifyListeners();
  }

  Future<void> setMode(bool usePhoto) async {
    if (!connected || photoMode == usePhoto) return;
    await _call('stopViewFinder');
    await _disposePreview();
    previewLoading = true;
    notifyListeners();
    if (await _call('setMode', {'mode': usePhoto ? 'photo' : 'video'})) {
      photoMode = usePhoto;
      settings['systemMode'] = usePhoto ? 'Capture' : 'Record';
      notifyListeners();
      await Future<void>.delayed(const Duration(milliseconds: 550));
      await startViewfinder();
    } else {
      previewLoading = false;
      notifyListeners();
    }
  }

  Future<void> shutter() async {
    if (photoMode) {
      _pendingAutoDownload = autoDownload;
      if (await _call('capturePhoto')) {
        notice = autoDownload ? '照片已拍攝，完成後會套用底片與修圖' : '照片已拍攝並儲存在相機記憶卡';
      } else {
        _pendingAutoDownload = false;
      }
    } else {
      if (await _call(recording ? 'stopRecording' : 'startRecording')) {
        recording = !recording;
        await _restartViewfinder(
          stopFirst: false,
          delay: const Duration(milliseconds: 650),
        );
      }
    }
    notifyListeners();
  }

  Future<void> _finishCapturedPhoto() async {
    if (_handlingCapture) return;
    _handlingCapture = true;
    _pendingAutoDownload = false;
    await Future<void>.delayed(const Duration(milliseconds: 900));
    await loadFiles();
    Map<String, Object?>? latest;
    for (final file in files) {
      final name = file['name']?.toString().toLowerCase() ?? '';
      if (name.endsWith('.jpg') || name.endsWith('.jpeg')) {
        latest = file;
        break;
      }
    }
    if (latest != null) {
      await downloadFile(latest['name'].toString());
    } else {
      notice = '已拍照，但相機尚未回傳新照片；可到相簿重新整理';
    }
    _handlingCapture = false;
    notifyListeners();
  }

  Future<void> startViewfinder() async {
    previewLoading = true;
    notifyListeners();
    if (!await _call('startViewFinder')) {
      previewLoading = false;
      notifyListeners();
      return;
    }
    final url = await _methods.invokeMethod<String>('getRtspUrl');
    if (url != null) await _startPreview(url);
  }

  Future<void> _restartViewfinder({
    bool stopFirst = true,
    Duration delay = const Duration(milliseconds: 450),
  }) async {
    if (!connected || _restartingPreview) return;
    _restartingPreview = true;
    if (stopFirst) {
      try {
        await _methods.invokeMethod('stopViewFinder');
      } catch (_) {}
    }
    await _disposePreview();
    previewLoading = true;
    notifyListeners();
    await Future<void>.delayed(delay);
    await startViewfinder();
    _restartingPreview = false;
  }

  Future<void> updateSetting({
    required String method,
    required String key,
    required Object value,
    String? message,
  }) async {
    final argumentKey = value is bool ? 'enabled' : 'value';
    if (await _call(method, {argumentKey: value})) {
      settings[key] = switch (key) {
        'microphone' => value == true ? 'Off' : 'On',
        'stabilization' => value == true ? 'On' : 'Off',
        _ => value,
      };
      notice = message ?? '設定已套用到相機';
      notifyListeners();
    }
  }

  Future<void> loadFiles() async {
    try {
      final raw = await _methods.invokeListMethod<Object?>('getFiles') ?? [];
      files =
          raw
              .whereType<Map>()
              .map((item) => Map<String, Object?>.from(item))
              .toList()
            ..sort(
              (a, b) => ((b['time'] as int?) ?? 0).compareTo(
                (a['time'] as int?) ?? 0,
              ),
            );
      error = null;
    } catch (exception) {
      _setError(exception);
    }
    notifyListeners();
  }

  Future<bool> downloadFile(String name, {bool fromBulk = false}) async {
    if (downloading.contains(name)) return false;
    downloading.add(name);
    downloadProgress[name] = 0;
    if (!fromBulk) notice = '正在下載 ${displayName(name)}';
    notifyListeners();
    try {
      await _methods.invokeMethod('downloadFile', {
        'name': name,
        'crop2x': crop2x,
        'filter': isPhoto(name) ? filmFilter : 'none',
        'enhance': isPhoto(name) && enhancePhotos,
        'matrix': isPhoto(name)
            ? _currentFilmMatrix(this)
            : _filmMatrix('none'),
        'vignette': isPhoto(name) ? _currentVignette(this) : 0,
      });
      downloaded.add(name);
      if (!fromBulk) notice = '已下載到手機相簿';
      error = null;
      return true;
    } catch (exception) {
      _setError(exception);
      return false;
    } finally {
      downloading.remove(name);
      downloadProgress.remove(name);
      notifyListeners();
    }
  }

  Future<void> downloadAll([Iterable<String>? names]) async {
    if (downloadingAll || files.isEmpty) return;
    final queue = (names ?? visibleFiles.map((item) => item['name'].toString()))
        .where((name) => !downloaded.contains(name))
        .toList();
    if (queue.isEmpty) {
      notice = '相簿內的檔案都已下載';
      notifyListeners();
      return;
    }
    downloadingAll = true;
    bulkCompleted = 0;
    bulkTotal = queue.length;
    notice = '準備下載 $bulkTotal 個檔案';
    notifyListeners();
    var succeeded = 0;
    for (final name in queue) {
      if (!connected) break;
      final ok = await downloadFile(name, fromBulk: true);
      bulkCompleted++;
      if (ok) succeeded++;
      notice = '全部下載：$bulkCompleted / $bulkTotal';
      notifyListeners();
    }
    downloadingAll = false;
    notice = '已下載 $succeeded 個檔案到手機相簿';
    notifyListeners();
  }

  Future<void> deleteFile(String name) async {
    if (await _call('deleteFile', {'name': name})) {
      files.removeWhere((item) => item['name'] == name);
      notice = '已從相機刪除檔案';
      notifyListeners();
    }
  }

  String mediaUrl(String name) {
    final cleanName = displayName(name);
    return Uri(
      scheme: 'http',
      host: '192.168.42.1',
      pathSegments: ['DCIM', '100MEDIA', cleanName],
    ).toString();
  }

  Future<Uint8List?> thumbnail(String name) =>
      _thumbnailRequests.putIfAbsent(name, () async {
        try {
          return await _methods.invokeMethod<Uint8List>('getThumbnail', {
            'name': name,
          });
        } catch (_) {
          return null;
        }
      });

  String displayName(String name) => name.replaceAll('\\', '/').split('/').last;

  bool isPhoto(String name) {
    final lower = displayName(name).toLowerCase();
    return lower.endsWith('.jpg') ||
        lower.endsWith('.jpeg') ||
        lower.endsWith('.png');
  }

  bool isVideo(String name) {
    final lower = displayName(name).toLowerCase();
    return lower.endsWith('.mp4') || lower.endsWith('.mov');
  }

  List<Map<String, Object?>> get visibleFiles => files.where((file) {
    final name = file['name']?.toString() ?? '';
    return switch (galleryCategory) {
      'photos' => isPhoto(name),
      'videos' => isVideo(name),
      _ => !isPhoto(name) && !isVideo(name),
    };
  }).toList();

  void setGalleryCategory(String value) {
    galleryCategory = value;
    selectedFiles.clear();
    notifyListeners();
  }

  void toggleSelection(String name) {
    selectedFiles.contains(name)
        ? selectedFiles.remove(name)
        : selectedFiles.add(name);
    notifyListeners();
  }

  void clearSelection() {
    selectedFiles.clear();
    notifyListeners();
  }

  void selectAllVisible() {
    selectedFiles.addAll(
      visibleFiles.map((file) => file['name']?.toString()).whereType<String>(),
    );
    notifyListeners();
  }

  Future<bool> _call(String method, [Map<String, Object?>? arguments]) async {
    try {
      error = null;
      await _methods.invokeMethod(method, arguments);
      return true;
    } catch (exception) {
      _setError(exception);
      return false;
    }
  }

  Future<void> _startPreview(String url) async {
    if (preview != null) {
      previewLoading = false;
      notifyListeners();
      return;
    }
    try {
      final player = VideoPlayerController.networkUrl(
        Uri.parse(url),
        videoPlayerOptions: VideoPlayerOptions(mixWithOthers: true),
      );
      await player.initialize().timeout(const Duration(seconds: 10));
      await player.setVolume(0);
      await player.play();
      preview = player;
      _startPreviewWatchdog();
      error = null;
    } catch (_) {
      error = '相機控制已連線，但即時影像串流無法播放；控制功能仍可使用。';
    }
    previewLoading = false;
    notifyListeners();
  }

  void _startPreviewWatchdog() {
    _previewWatchdog?.cancel();
    _lastPreviewPosition = Duration.zero;
    _previewStalls = 0;
    _previewWatchdog = Timer.periodic(const Duration(seconds: 2), (_) {
      final player = preview;
      if (!connected || player == null || !player.value.isPlaying) return;
      final position = player.value.position;
      if (position <= _lastPreviewPosition) {
        _previewStalls++;
      } else {
        _previewStalls = 0;
      }
      _lastPreviewPosition = position;
      if (_previewStalls >= 3 && !_restartingPreview) {
        _previewStalls = 0;
        notice = '預覽停滯，正在重新載入即時畫面';
        _restartViewfinder(delay: const Duration(milliseconds: 180));
      }
    });
  }

  void _setError(Object? value) {
    if (value is PlatformException) {
      error = value.message ?? value.code;
    } else {
      error = value?.toString();
    }
    connecting = false;
    notifyListeners();
  }

  void clearMessage() {
    error = null;
    notice = null;
    notifyListeners();
  }

  Future<void> _disposePreview() async {
    _previewWatchdog?.cancel();
    final old = preview;
    preview = null;
    await old?.dispose();
  }

  @override
  void dispose() {
    _reconnectTimer?.cancel();
    _previewWatchdog?.cancel();
    _subscription?.cancel();
    _disposePreview();
    super.dispose();
  }
}

class RemoteShell extends StatefulWidget {
  const RemoteShell({super.key});

  @override
  State<RemoteShell> createState() => _RemoteShellState();
}

class _RemoteShellState extends State<RemoteShell> {
  late final YiCameraController camera;
  int page = 0;

  @override
  void initState() {
    super.initState();
    camera = YiCameraController()..addListener(_changed);
  }

  void _changed() {
    if (mounted) setState(() {});
  }

  @override
  void dispose() {
    camera
      ..removeListener(_changed)
      ..dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Column(
          children: [
            Expanded(
              child: IndexedStack(
                index: page,
                children: [
                  LivePage(camera: camera),
                  FilmGalleryPage(camera: camera),
                  CameraSettingsPage(camera: camera),
                ],
              ),
            ),
            if (camera.error != null || camera.notice != null)
              _MessageBar(
                message: camera.error ?? camera.notice!,
                isError: camera.error != null,
                onClose: camera.clearMessage,
              ),
          ],
        ),
      ),
      bottomNavigationBar: NavigationBar(
        selectedIndex: page,
        onDestinationSelected: (value) {
          setState(() => page = value);
          if (value == 1 && camera.connected) camera.loadFiles();
        },
        destinations: const [
          NavigationDestination(
            icon: Icon(Icons.videocam_outlined),
            selectedIcon: Icon(Icons.videocam),
            label: '遙控',
          ),
          NavigationDestination(
            icon: Icon(Icons.grid_view_rounded),
            selectedIcon: Icon(Icons.grid_view),
            label: '相簿',
          ),
          NavigationDestination(icon: Icon(Icons.tune_rounded), label: '設定'),
        ],
      ),
    );
  }
}

class LivePage extends StatelessWidget {
  const LivePage({super.key, required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: [
        SliverToBoxAdapter(
          child: _PageHeader(
            camera: camera,
            title: 'LUMO',
            subtitle: 'YI 4K REMOTE',
          ),
        ),
        SliverPadding(
          padding: const EdgeInsets.fromLTRB(16, 4, 16, 24),
          sliver: SliverList.list(
            children: camera.connected
                ? _connected(context)
                : _disconnected(context),
          ),
        ),
      ],
    );
  }

  List<Widget> _disconnected(BuildContext context) => [
    const SizedBox(height: 54),
    const Text(
      'YI 4K ACTION CAMERA',
      style: TextStyle(fontSize: 12, letterSpacing: 1.7, color: Colors.white38),
    ),
    const SizedBox(height: 14),
    const Text(
      '連接相機',
      style: TextStyle(fontSize: 30, fontWeight: FontWeight.w500),
    ),
    const SizedBox(height: 34),
    const _ConnectionStep(number: '01', text: '在相機上開啟 Wi‑Fi'),
    const Divider(height: 1),
    const _ConnectionStep(number: '02', text: '手機連接 YDXJ_… 網路'),
    const Divider(height: 1),
    const _ConnectionStep(number: '03', text: '回到這裡按下連接'),
    const SizedBox(height: 34),
    SizedBox(
      width: double.infinity,
      height: 54,
      child: FilledButton(
        onPressed: camera.connecting ? null : camera.connect,
        child: Text(camera.connecting ? '正在連接…' : '連接相機'),
      ),
    ),
    const SizedBox(height: 6),
    Align(
      alignment: Alignment.centerLeft,
      child: TextButton(
        onPressed: camera.openWifiSettings,
        child: const Text('開啟手機 Wi‑Fi 設定'),
      ),
    ),
    const Padding(
      padding: EdgeInsets.only(top: 18),
      child: Text(
        '連上相機後顯示「沒有網際網路」是正常的。',
        style: TextStyle(color: Colors.white30, fontSize: 12),
      ),
    ),
  ];

  List<Widget> _connected(BuildContext context) => [
    _PreviewCard(camera: camera),
    const SizedBox(height: 14),
    _QuickControls(camera: camera),
    const SizedBox(height: 14),
    _CapturePanel(camera: camera),
  ];
}

class _ConnectionStep extends StatelessWidget {
  const _ConnectionStep({required this.number, required this.text});

  final String number;
  final String text;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 18),
      child: Row(
        children: [
          SizedBox(
            width: 42,
            child: Text(
              number,
              style: const TextStyle(color: _accent, letterSpacing: 1),
            ),
          ),
          Text(text, style: const TextStyle(fontSize: 16)),
        ],
      ),
    );
  }
}

class _PreviewCard extends StatelessWidget {
  const _PreviewCard({required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return RepaintBoundary(
      child: ClipRRect(
        borderRadius: BorderRadius.circular(4),
        child: AspectRatio(
          aspectRatio: 16 / 9,
          child: Stack(
            fit: StackFit.expand,
            children: [
              const ColoredBox(color: Colors.black),
              if (camera.preview?.value.isInitialized == true)
                ClipRect(
                  child: ColorFiltered(
                    colorFilter: ColorFilter.matrix(_currentFilmMatrix(camera)),
                    child: Transform.scale(
                      scale: camera.previewCrop2x ? 2 : 1,
                      child: FittedBox(
                        fit: BoxFit.contain,
                        child: SizedBox(
                          width: camera.preview!.value.size.width,
                          height: camera.preview!.value.size.height,
                          child: VideoPlayer(camera.preview!),
                        ),
                      ),
                    ),
                  ),
                )
              else
                _PreviewPlaceholder(loading: camera.previewLoading),
              const DecoratedBox(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topCenter,
                    end: Alignment.bottomCenter,
                    colors: [
                      Color(0x88000000),
                      Colors.transparent,
                      Color(0x66000000),
                    ],
                    stops: [0, .42, 1],
                  ),
                ),
              ),
              Positioned(
                left: 14,
                top: 14,
                child: _Pill(
                  icon: camera.recording
                      ? Icons.fiber_manual_record
                      : Icons.wifi_rounded,
                  label: camera.recording ? 'REC' : 'LIVE',
                  color: camera.recording
                      ? Colors.redAccent
                      : const Color(0xFF56E39F),
                ),
              ),
              Positioned(
                right: 14,
                top: 14,
                child: _Pill(
                  icon: Icons.battery_5_bar_rounded,
                  label: camera.battery == null ? '--%' : '${camera.battery}%',
                ),
              ),
              Positioned(
                left: 12,
                bottom: 10,
                child: Text(
                  camera.photoMode
                      ? '${camera.previewCrop2x ? '2×' : '1×'}  ·  ${_filmLabel(camera.filmFilter)}'
                      : _resolutionLabel(
                          camera.settings['videoResolution']?.toString(),
                        ),
                  style: const TextStyle(fontSize: 11, color: Colors.white70),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _PreviewPlaceholder extends StatelessWidget {
  const _PreviewPlaceholder({required this.loading});

  final bool loading;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (loading)
            const SizedBox.square(
              dimension: 30,
              child: CircularProgressIndicator(strokeWidth: 2.5),
            )
          else
            const Icon(
              Icons.videocam_off_outlined,
              size: 36,
              color: Colors.white38,
            ),
          const SizedBox(height: 10),
          Text(
            loading ? '正在載入低延遲預覽' : '預覽尚未啟動',
            style: const TextStyle(color: Colors.white54),
          ),
        ],
      ),
    );
  }
}

class _QuickControls extends StatelessWidget {
  const _QuickControls({required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    if (camera.photoMode) {
      return SizedBox(
        height: 52,
        child: ListView(
          scrollDirection: Axis.horizontal,
          children: [
            _QuickChip(
              icon: camera.previewCrop2x ? Icons.zoom_in : Icons.crop_free,
              label: camera.previewCrop2x ? '預覽 2×' : '預覽原圖',
              onTap: () => camera.setLocalPreference(
                'previewCrop2x',
                !camera.previewCrop2x,
              ),
            ),
            for (final entry in _filmFilters.entries)
              _FilmChip(
                label: entry.value,
                selected: camera.filmFilter == entry.key,
                onTap: () => camera.setLocalPreference('filter', entry.key),
              ),
          ],
        ),
      );
    }
    return SizedBox(
      height: 44,
      child: ListView(
        scrollDirection: Axis.horizontal,
        children: [
          _QuickChip(
            icon: Icons.hd_rounded,
            label: _resolutionLabel(
              camera.settings['videoResolution']?.toString(),
            ),
            onTap: () => _showResolutionPicker(context, camera),
          ),
          _QuickChip(
            icon: Icons.panorama_wide_angle_rounded,
            label: _labelFor(camera.settings['fieldOfView'], const {
              'Wide': '廣角',
              'Medium': '中等',
              'Narrow': '窄角',
            }),
            onTap: () => _showChoicePicker(
              context,
              camera: camera,
              title: '視角',
              method: 'setFieldOfView',
              keyName: 'fieldOfView',
              choices: const {'Wide': '廣角', 'Medium': '中等', 'Narrow': '窄角'},
            ),
          ),
          _QuickChip(
            icon: Icons.auto_awesome_rounded,
            label: _labelFor(camera.settings['videoQuality'], const {
              'Low': '省空間',
              'Middle': '標準',
              'High': '高畫質',
            }),
            onTap: () => _showChoicePicker(
              context,
              camera: camera,
              title: '錄影畫質',
              method: 'setQuality',
              keyName: 'videoQuality',
              choices: const {'Low': '省空間', 'Middle': '標準', 'High': '高畫質'},
            ),
          ),
        ],
      ),
    );
  }
}

class _QuickChip extends StatelessWidget {
  const _QuickChip({
    required this.icon,
    required this.label,
    required this.onTap,
  });

  final IconData icon;
  final String label;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(right: 8),
      child: ActionChip(
        avatar: Icon(icon, size: 17, color: _accent),
        label: Text(label),
        backgroundColor: _surface,
        side: const BorderSide(color: Colors.white10),
        onPressed: onTap,
      ),
    );
  }
}

class _FilmChip extends StatelessWidget {
  const _FilmChip({
    required this.label,
    required this.selected,
    required this.onTap,
  });

  final String label;
  final bool selected;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(right: 7),
      child: ChoiceChip(
        label: Text(label),
        selected: selected,
        showCheckmark: false,
        onSelected: (_) => onTap(),
      ),
    );
  }
}

class _CapturePanel extends StatelessWidget {
  const _CapturePanel({required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(16, 12, 16, 20),
        child: Column(
          children: [
            SegmentedButton<bool>(
              showSelectedIcon: false,
              segments: const [
                ButtonSegment(
                  value: false,
                  icon: Icon(Icons.videocam_rounded),
                  label: Text('錄影'),
                ),
                ButtonSegment(
                  value: true,
                  icon: Icon(Icons.photo_camera_rounded),
                  label: Text('拍照'),
                ),
              ],
              selected: {camera.photoMode},
              onSelectionChanged: (value) => camera.setMode(value.first),
            ),
            const SizedBox(height: 20),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                _RoundButton(
                  icon: Icons.sync_rounded,
                  label: '同步',
                  onTap: camera.refresh,
                ),
                GestureDetector(
                  onTap: camera.shutter,
                  child: AnimatedContainer(
                    duration: const Duration(milliseconds: 160),
                    width: 82,
                    height: 82,
                    padding: const EdgeInsets.all(6),
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      border: Border.all(color: Colors.white, width: 3),
                    ),
                    child: Center(
                      child: AnimatedContainer(
                        duration: const Duration(milliseconds: 160),
                        width: camera.recording ? 34 : 64,
                        height: camera.recording ? 34 : 64,
                        decoration: BoxDecoration(
                          color: camera.photoMode ? Colors.white : _accent,
                          shape: camera.recording
                              ? BoxShape.rectangle
                              : BoxShape.circle,
                          borderRadius: camera.recording
                              ? BorderRadius.circular(8)
                              : null,
                        ),
                      ),
                    ),
                  ),
                ),
                _RoundButton(
                  icon: Icons.link_off_rounded,
                  label: '中斷',
                  onTap: camera.disconnect,
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class FilmGalleryPage extends StatelessWidget {
  const FilmGalleryPage({super.key, required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    final visible = camera.visibleFiles;
    final selecting = camera.selectedFiles.isNotEmpty;
    return Column(
      children: [
        if (selecting)
          AppBar(
            automaticallyImplyLeading: false,
            title: Text('已選 ${camera.selectedFiles.length} 個'),
            leading: IconButton(
              onPressed: camera.clearSelection,
              icon: const Icon(Icons.close),
            ),
            actions: [
              TextButton(
                onPressed: camera.selectAllVisible,
                child: const Text('全選'),
              ),
              IconButton(
                tooltip: '下載所選',
                onPressed: camera.downloadingAll
                    ? null
                    : () => camera.downloadAll(camera.selectedFiles),
                icon: const Icon(Icons.download_rounded),
              ),
            ],
          )
        else
          _PageHeader(
            camera: camera,
            title: '相簿',
            subtitle: camera.connected ? '${camera.files.length} 個檔案' : '尚未連接',
          ),
        Padding(
          padding: const EdgeInsets.fromLTRB(12, 0, 12, 10),
          child: Row(
            children: [
              Expanded(
                child: SegmentedButton<String>(
                  showSelectedIcon: false,
                  segments: const [
                    ButtonSegment(value: 'photos', label: Text('照片')),
                    ButtonSegment(value: 'videos', label: Text('MP4')),
                    ButtonSegment(value: 'other', label: Text('其他')),
                  ],
                  selected: {camera.galleryCategory},
                  onSelectionChanged: (values) =>
                      camera.setGalleryCategory(values.first),
                ),
              ),
              const SizedBox(width: 6),
              IconButton(
                tooltip: '重新整理',
                onPressed: camera.connected ? camera.loadFiles : null,
                icon: const Icon(Icons.refresh),
              ),
            ],
          ),
        ),
        if (camera.galleryCategory == 'photos')
          Padding(
            padding: const EdgeInsets.fromLTRB(14, 0, 14, 8),
            child: Row(
              children: [
                TextButton.icon(
                  onPressed: () =>
                      camera.setLocalPreference('crop2x', !camera.crop2x),
                  icon: Icon(camera.crop2x ? Icons.zoom_in : Icons.crop_free),
                  label: Text(camera.crop2x ? '2× 裁切' : '保留原圖'),
                ),
                const Spacer(),
                Text(
                  '長按可多選',
                  style: Theme.of(
                    context,
                  ).textTheme.bodySmall?.copyWith(color: Colors.white38),
                ),
              ],
            ),
          ),
        Expanded(
          child: !camera.connected
              ? const _EmptyState(
                  icon: Icons.link_off,
                  title: '先連接相機',
                  subtitle: '連線後可瀏覽記憶卡內容',
                )
              : visible.isEmpty
              ? _EmptyState(
                  icon: camera.galleryCategory == 'other'
                      ? Icons.insert_drive_file_outlined
                      : Icons.photo_library_outlined,
                  title: '這個分類沒有檔案',
                  subtitle: camera.galleryCategory == 'other'
                      ? 'MEC、THM 等相機輔助檔會顯示在這裡'
                      : '下拉或按重新整理讀取相機',
                )
              : RefreshIndicator(
                  onRefresh: camera.loadFiles,
                  child: GridView.builder(
                    padding: const EdgeInsets.fromLTRB(2, 2, 2, 24),
                    cacheExtent: 0,
                    gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                      crossAxisCount: camera.galleryCategory == 'photos'
                          ? 3
                          : 2,
                      crossAxisSpacing: 2,
                      mainAxisSpacing: 2,
                      childAspectRatio: camera.galleryCategory == 'photos'
                          ? 1
                          : .92,
                    ),
                    itemCount: visible.length,
                    itemBuilder: (context, index) =>
                        _FilmMediaTile(camera: camera, file: visible[index]),
                  ),
                ),
        ),
        if (camera.connected && visible.isNotEmpty && !selecting)
          SafeArea(
            top: false,
            child: Padding(
              padding: const EdgeInsets.fromLTRB(14, 6, 14, 10),
              child: SizedBox(
                width: double.infinity,
                child: FilledButton.icon(
                  onPressed: camera.downloadingAll
                      ? null
                      : () => camera.downloadAll(),
                  icon: camera.downloadingAll
                      ? const SizedBox.square(
                          dimension: 16,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.download_rounded),
                  label: Text(
                    camera.downloadingAll
                        ? '${camera.bulkCompleted} / ${camera.bulkTotal}'
                        : '下載此分類全部檔案',
                  ),
                ),
              ),
            ),
          ),
      ],
    );
  }
}

class _FilmMediaTile extends StatelessWidget {
  const _FilmMediaTile({required this.camera, required this.file});

  final YiCameraController camera;
  final Map<String, Object?> file;

  @override
  Widget build(BuildContext context) {
    final name = file['name']?.toString() ?? '';
    final photo = camera.isPhoto(name);
    final video = camera.isVideo(name);
    final selected = camera.selectedFiles.contains(name);
    final child = Stack(
      fit: StackFit.expand,
      children: [
        if (photo)
          ClipRect(
            child: ColorFiltered(
              colorFilter: ColorFilter.matrix(_currentFilmMatrix(camera)),
              child: Transform.scale(
                scale: camera.crop2x ? 2 : 1,
                child: FutureBuilder<Uint8List?>(
                  future: camera.thumbnail(name),
                  builder: (context, snapshot) {
                    final bytes = snapshot.data;
                    if (bytes != null && bytes.isNotEmpty) {
                      return Image.memory(
                        bytes,
                        fit: BoxFit.cover,
                        cacheWidth: 240,
                        filterQuality: FilterQuality.low,
                      );
                    }
                    if (snapshot.connectionState == ConnectionState.waiting) {
                      return const ColoredBox(color: _surface);
                    }
                    return Image.network(
                      camera.mediaUrl(name),
                      fit: BoxFit.cover,
                      cacheWidth: 240,
                      filterQuality: FilterQuality.low,
                      errorBuilder: (_, _, _) =>
                          const _ThumbnailFallback(isVideo: false),
                    );
                  },
                ),
              ),
            ),
          )
        else
          ColoredBox(
            color: _surface,
            child: Icon(
              video ? Icons.play_circle_outline : Icons.description_outlined,
              size: 38,
              color: Colors.white38,
            ),
          ),
        if (!photo)
          Positioned(
            left: 8,
            right: 8,
            bottom: 7,
            child: Text(
              camera.displayName(name),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(fontSize: 11, color: Colors.white70),
            ),
          ),
        if (selected) const ColoredBox(color: Color(0x55332211)),
        if (selected)
          const Positioned(
            right: 7,
            top: 7,
            child: Icon(Icons.check_circle, color: _accent),
          ),
        if (camera.downloading.contains(name))
          Center(
            child: CircularProgressIndicator(
              value: camera.downloadProgress[name],
            ),
          ),
      ],
    );
    return GestureDetector(
      onLongPress: () => camera.toggleSelection(name),
      onTap: () {
        if (camera.selectedFiles.isNotEmpty) {
          camera.toggleSelection(name);
        } else if (photo) {
          Navigator.of(context).push(
            MaterialPageRoute<void>(
              builder: (_) => _PhotoViewer(camera: camera, name: name),
            ),
          );
        } else {
          camera.downloadFile(name);
        }
      },
      child: child,
    );
  }
}

class _PhotoViewer extends StatelessWidget {
  const _PhotoViewer({required this.camera, required this.name});

  final YiCameraController camera;
  final String name;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        backgroundColor: Colors.black,
        title: Text(camera.displayName(name)),
        actions: [
          IconButton(
            tooltip: '下載',
            onPressed: () => camera.downloadFile(name),
            icon: const Icon(Icons.download),
          ),
        ],
      ),
      body: InteractiveViewer(
        minScale: 1,
        maxScale: 5,
        child: Center(
          child: ClipRect(
            child: ColorFiltered(
              colorFilter: ColorFilter.matrix(_currentFilmMatrix(camera)),
              child: Transform.scale(
                scale: camera.crop2x ? 2 : 1,
                child: Image.network(
                  camera.mediaUrl(name),
                  fit: BoxFit.contain,
                ),
              ),
            ),
          ),
        ),
      ),
      bottomNavigationBar: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Row(
            children: [
              Expanded(
                child: OutlinedButton(
                  onPressed: () =>
                      camera.setLocalPreference('crop2x', !camera.crop2x),
                  child: Text(camera.crop2x ? '顯示原圖' : '顯示 2× 裁切'),
                ),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: FilledButton(
                  onPressed: () => camera.downloadFile(name),
                  child: const Text('下載照片'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class GalleryPage extends StatelessWidget {
  const GalleryPage({super.key, required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        _PageHeader(
          camera: camera,
          title: '相機相簿',
          subtitle: camera.connected ? '${camera.files.length} 個項目' : '尚未連接',
        ),
        if (camera.connected && camera.files.isNotEmpty)
          Padding(
            padding: const EdgeInsets.fromLTRB(14, 0, 14, 12),
            child: Row(
              children: [
                Expanded(
                  child: FilledButton.tonalIcon(
                    onPressed: camera.downloadingAll
                        ? null
                        : camera.downloadAll,
                    icon: camera.downloadingAll
                        ? const SizedBox.square(
                            dimension: 16,
                            child: CircularProgressIndicator(strokeWidth: 2),
                          )
                        : const Icon(Icons.download_for_offline_rounded),
                    label: Text(
                      camera.downloadingAll
                          ? '下載中 ${camera.bulkCompleted}/${camera.bulkTotal}'
                          : '全部下載',
                    ),
                  ),
                ),
                const SizedBox(width: 8),
                IconButton.filledTonal(
                  onPressed: camera.loadFiles,
                  icon: const Icon(Icons.refresh_rounded),
                  tooltip: '重新整理',
                ),
              ],
            ),
          ),
        Expanded(child: _content(context)),
      ],
    );
  }

  Widget _content(BuildContext context) {
    if (!camera.connected) {
      return const _EmptyState(
        icon: Icons.cloud_off_rounded,
        title: '先連接相機',
        subtitle: '連線後才能瀏覽記憶卡裡的照片與影片',
      );
    }
    if (camera.files.isEmpty) {
      return _EmptyState(
        icon: Icons.photo_library_outlined,
        title: '還沒有讀取到檔案',
        subtitle: '按下方按鈕重新整理相機記憶卡',
        action: FilledButton.tonalIcon(
          onPressed: camera.loadFiles,
          icon: const Icon(Icons.refresh),
          label: const Text('讀取相簿'),
        ),
      );
    }
    return RefreshIndicator(
      onRefresh: camera.loadFiles,
      child: GridView.builder(
        padding: const EdgeInsets.fromLTRB(14, 4, 14, 24),
        gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: 2,
          crossAxisSpacing: 10,
          mainAxisSpacing: 10,
          childAspectRatio: .72,
        ),
        itemCount: camera.files.length,
        itemBuilder: (context, index) =>
            _MediaTile(camera: camera, file: camera.files[index]),
      ),
    );
  }
}

class _MediaTile extends StatelessWidget {
  const _MediaTile({required this.camera, required this.file});

  final YiCameraController camera;
  final Map<String, Object?> file;

  @override
  Widget build(BuildContext context) {
    final name = file['name']?.toString() ?? '未命名檔案';
    final displayName = camera.displayName(name);
    final isVideo = displayName.toLowerCase().endsWith('.mp4');
    final isDownloading = camera.downloading.contains(name);
    final isDownloaded = camera.downloaded.contains(name);
    final progress = camera.downloadProgress[name];
    return Card(
      clipBehavior: Clip.antiAlias,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Expanded(
            child: Stack(
              fit: StackFit.expand,
              children: [
                if (!isVideo)
                  Image.network(
                    camera.mediaUrl(name),
                    fit: BoxFit.cover,
                    cacheWidth: 420,
                    filterQuality: FilterQuality.low,
                    gaplessPlayback: true,
                    errorBuilder: (_, _, _) =>
                        const _ThumbnailFallback(isVideo: false),
                  )
                else
                  const _ThumbnailFallback(isVideo: true),
                const DecoratedBox(
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      begin: Alignment.topCenter,
                      end: Alignment.bottomCenter,
                      colors: [Colors.transparent, Color(0xAA000000)],
                      stops: [.55, 1],
                    ),
                  ),
                ),
                Positioned(
                  left: 9,
                  bottom: 8,
                  child: _Pill(
                    icon: isVideo
                        ? Icons.play_arrow_rounded
                        : Icons.photo_rounded,
                    label: isVideo ? 'VIDEO' : 'PHOTO',
                  ),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(11, 10, 7, 8),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  displayName,
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    fontWeight: FontWeight.w700,
                    fontSize: 13,
                  ),
                ),
                const SizedBox(height: 3),
                Text(
                  _fileSize(file['size'] as int? ?? 0),
                  style: const TextStyle(color: Colors.white38, fontSize: 11),
                ),
                const SizedBox(height: 5),
                Row(
                  children: [
                    Expanded(
                      child: TextButton.icon(
                        onPressed: isDownloading
                            ? null
                            : () => camera.downloadFile(name),
                        icon: isDownloading
                            ? SizedBox.square(
                                dimension: 15,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                  value: progress,
                                ),
                              )
                            : Icon(
                                isDownloaded
                                    ? Icons.check_circle_rounded
                                    : Icons.download_rounded,
                                size: 18,
                              ),
                        label: Text(
                          isDownloading
                              ? progress == null
                                    ? '下載中'
                                    : '${(progress * 100).round()}%'
                              : isDownloaded
                              ? '已下載'
                              : '下載',
                        ),
                      ),
                    ),
                    IconButton(
                      visualDensity: VisualDensity.compact,
                      onPressed: () =>
                          _confirmDelete(context, name, displayName),
                      icon: const Icon(
                        Icons.delete_outline_rounded,
                        size: 20,
                        color: Colors.white54,
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Future<void> _confirmDelete(
    BuildContext context,
    String name,
    String displayName,
  ) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('從相機刪除？'),
        content: Text('$displayName\n刪除後無法復原。'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('取消'),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text('刪除'),
          ),
        ],
      ),
    );
    if (confirmed == true) camera.deleteFile(name);
  }
}

class _ThumbnailFallback extends StatelessWidget {
  const _ThumbnailFallback({required this.isVideo});

  final bool isVideo;

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: const BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [Color(0xFF292D34), Color(0xFF101216)],
        ),
      ),
      child: Center(
        child: Icon(
          isVideo ? Icons.movie_outlined : Icons.image_outlined,
          size: 42,
          color: Colors.white24,
        ),
      ),
    );
  }
}

class CameraSettingsPage extends StatelessWidget {
  const CameraSettingsPage({super.key, required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: [
        SliverToBoxAdapter(
          child: _PageHeader(
            camera: camera,
            title: '相機設定',
            subtitle: camera.connected ? '變更會立即寫入相機' : '連接後可調整',
          ),
        ),
        SliverPadding(
          padding: const EdgeInsets.fromLTRB(16, 4, 16, 28),
          sliver: SliverList.list(
            children: [
              _DeviceCard(camera: camera),
              const SizedBox(height: 14),
              _SettingsGroup(
                title: '拍照與底片',
                icon: Icons.photo_camera_outlined,
                children: [
                  _ChoiceSetting(
                    title: '照片解析度',
                    value: camera.settings['photoResolution']?.toString(),
                    labels: const {
                      'p_12MP_4000x3000_4x3_w': '12MP · 最高畫質',
                      'p_8MP_3840x2160_16x9_w': '8MP · 16:9',
                      'p_7MP_3008x2256_4x3_w': '7MP · 4:3',
                    },
                    enabled: camera.connected,
                    onChanged: (value) => camera.updateSetting(
                      method: 'setPhotoResolution',
                      key: 'photoResolution',
                      value: value,
                    ),
                  ),
                  _ChoiceSetting(
                    title: '底片色調',
                    value: camera.filmFilter,
                    labels: _filmFilters,
                    enabled: true,
                    onChanged: (value) =>
                        camera.setLocalPreference('filter', value),
                  ),
                  _CustomFilterSetting(camera: camera),
                  _SwitchSetting(
                    title: '輕量高畫質修圖',
                    subtitle: '改善對比與細節，JPEG 以 96% 品質儲存',
                    value: camera.enhancePhotos,
                    enabled: true,
                    onChanged: (value) =>
                        camera.setLocalPreference('enhance', value),
                  ),
                  _SwitchSetting(
                    title: '下載時中央 2× 裁切',
                    subtitle: '關閉時完整保留相機原圖',
                    value: camera.crop2x,
                    enabled: true,
                    onChanged: (value) =>
                        camera.setLocalPreference('crop2x', value),
                  ),
                  _SwitchSetting(
                    title: '取景預覽 2×',
                    subtitle: '只改手機取景，不改相機鏡頭與原始檔',
                    value: camera.previewCrop2x,
                    enabled: true,
                    onChanged: (value) =>
                        camera.setLocalPreference('previewCrop2x', value),
                  ),
                  _SwitchSetting(
                    title: '拍照後自動存手機',
                    subtitle: '拍完自動下載並套用所選底片與修圖',
                    value: camera.autoDownload,
                    enabled: true,
                    onChanged: (value) =>
                        camera.setLocalPreference('autoDownload', value),
                  ),
                ],
              ),
              const SizedBox(height: 14),
              _SettingsGroup(
                title: '錄影',
                icon: Icons.videocam_rounded,
                children: [
                  _ChoiceSetting(
                    title: '解析度與幀率',
                    value: camera.settings['videoResolution']?.toString(),
                    labels: _resolutions,
                    enabled: camera.connected,
                    onChanged: (value) => camera.updateSetting(
                      method: 'setVideoResolution',
                      key: 'videoResolution',
                      value: value,
                    ),
                  ),
                  _ChoiceSetting(
                    title: '畫質',
                    value: camera.settings['videoQuality']?.toString(),
                    labels: const {'Low': '省空間', 'Middle': '標準', 'High': '高畫質'},
                    enabled: camera.connected,
                    onChanged: (value) => camera.updateSetting(
                      method: 'setQuality',
                      key: 'videoQuality',
                      value: value,
                    ),
                  ),
                  _ChoiceSetting(
                    title: '視角',
                    value: camera.settings['fieldOfView']?.toString(),
                    labels: const {
                      'Wide': '廣角',
                      'Medium': '中等',
                      'Narrow': '窄角',
                    },
                    enabled: camera.connected,
                    onChanged: (value) => camera.updateSetting(
                      method: 'setFieldOfView',
                      key: 'fieldOfView',
                      value: value,
                    ),
                  ),
                  _SwitchSetting(
                    title: '電子防手震',
                    subtitle: '部分高幀率模式可能不支援',
                    value: camera.settings['stabilization'] == 'On',
                    enabled: camera.connected,
                    onChanged: (value) => camera.updateSetting(
                      method: 'setStabilization',
                      key: 'stabilization',
                      value: value,
                    ),
                  ),
                  _SwitchSetting(
                    title: '收錄聲音',
                    subtitle: '關閉可錄製靜音影片',
                    value: camera.settings['microphone'] != 'On',
                    enabled: camera.connected,
                    onChanged: (value) => camera.updateSetting(
                      method: 'setMicrophone',
                      key: 'microphone',
                      value: value,
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 14),
              Card(
                child: ListTile(
                  contentPadding: const EdgeInsets.symmetric(
                    horizontal: 18,
                    vertical: 8,
                  ),
                  leading: const Icon(Icons.sync_rounded, color: _accent),
                  title: const Text('重新讀取相機設定'),
                  subtitle: const Text('從相機同步目前的所有參數'),
                  enabled: camera.connected,
                  trailing: const Icon(Icons.chevron_right),
                  onTap: camera.refresh,
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

class _DeviceCard extends StatelessWidget {
  const _DeviceCard({required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(18),
        child: Row(
          children: [
            Container(
              width: 54,
              height: 54,
              alignment: Alignment.center,
              decoration: BoxDecoration(
                color: const Color(0x22FF5A3C),
                borderRadius: BorderRadius.circular(17),
              ),
              child: const Text(
                'YI',
                style: TextStyle(
                  color: _accent,
                  fontSize: 20,
                  fontWeight: FontWeight.w900,
                ),
              ),
            ),
            const SizedBox(width: 14),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    camera.settings['productName']?.toString() ??
                        'YI 4K Action Camera',
                    style: const TextStyle(
                      fontWeight: FontWeight.w800,
                      fontSize: 16,
                    ),
                  ),
                  const SizedBox(height: 5),
                  Text(
                    camera.connected
                        ? '韌體 ${camera.settings['softwareVersion'] ?? '--'}  ·  電量 ${camera.battery ?? '--'}%'
                        : '尚未連接相機',
                    style: const TextStyle(color: Colors.white54, fontSize: 12),
                  ),
                ],
              ),
            ),
            Container(
              width: 9,
              height: 9,
              decoration: BoxDecoration(
                color: camera.connected
                    ? const Color(0xFF56E39F)
                    : Colors.white24,
                shape: BoxShape.circle,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _SettingsGroup extends StatelessWidget {
  const _SettingsGroup({
    required this.title,
    required this.icon,
    required this.children,
  });

  final String title;
  final IconData icon;
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(10, 16, 10, 8),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 8),
              child: Row(
                children: [
                  Icon(icon, color: _accent, size: 20),
                  const SizedBox(width: 9),
                  Text(
                    title,
                    style: const TextStyle(
                      fontWeight: FontWeight.w800,
                      fontSize: 16,
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 8),
            for (var i = 0; i < children.length; i++) ...[
              children[i],
              if (i != children.length - 1)
                const Divider(height: 1, indent: 8, endIndent: 8),
            ],
          ],
        ),
      ),
    );
  }
}

class _CustomFilterSetting extends StatelessWidget {
  const _CustomFilterSetting({required this.camera});

  final YiCameraController camera;

  @override
  Widget build(BuildContext context) {
    return ListTile(
      title: const Text('製作自己的濾鏡'),
      subtitle: const Text('調整色彩後儲存為「我的濾鏡」'),
      trailing: const Icon(Icons.tune),
      onTap: () => _openEditor(context),
    );
  }

  Future<void> _openEditor(BuildContext context) async {
    var contrast = camera.customContrast;
    var saturation = camera.customSaturation;
    var warmth = camera.customWarmth;
    var fade = camera.customFade;
    var vignette = camera.customVignette;
    final saved = await showModalBottomSheet<bool>(
      context: context,
      isScrollControlled: true,
      showDragHandle: true,
      builder: (context) => StatefulBuilder(
        builder: (context, setModalState) => SafeArea(
          child: Padding(
            padding: EdgeInsets.fromLTRB(
              18,
              0,
              18,
              18 + MediaQuery.viewInsetsOf(context).bottom,
            ),
            child: SingleChildScrollView(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    '我的濾鏡',
                    style: TextStyle(fontSize: 24, fontWeight: FontWeight.w500),
                  ),
                  const SizedBox(height: 8),
                  _FilterSlider(
                    label: '對比',
                    value: contrast,
                    min: .72,
                    max: 1.32,
                    onChanged: (value) => setModalState(() => contrast = value),
                  ),
                  _FilterSlider(
                    label: '飽和',
                    value: saturation,
                    min: .55,
                    max: 1.45,
                    onChanged: (value) =>
                        setModalState(() => saturation = value),
                  ),
                  _FilterSlider(
                    label: '色溫',
                    value: warmth,
                    min: -1,
                    max: 1,
                    onChanged: (value) => setModalState(() => warmth = value),
                  ),
                  _FilterSlider(
                    label: '褪色',
                    value: fade,
                    min: 0,
                    max: 1,
                    onChanged: (value) => setModalState(() => fade = value),
                  ),
                  _FilterSlider(
                    label: '暗角',
                    value: vignette,
                    min: 0,
                    max: .5,
                    onChanged: (value) => setModalState(() => vignette = value),
                  ),
                  const SizedBox(height: 12),
                  SizedBox(
                    width: double.infinity,
                    child: FilledButton(
                      onPressed: () => Navigator.pop(context, true),
                      child: const Text('儲存並套用'),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
    if (saved != true) return;
    await camera.setLocalPreference('customContrast', contrast);
    await camera.setLocalPreference('customSaturation', saturation);
    await camera.setLocalPreference('customWarmth', warmth);
    await camera.setLocalPreference('customFade', fade);
    await camera.setLocalPreference('customVignette', vignette);
    await camera.setLocalPreference('filter', 'custom');
  }
}

class _FilterSlider extends StatelessWidget {
  const _FilterSlider({
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.onChanged,
  });

  final String label;
  final double value;
  final double min;
  final double max;
  final ValueChanged<double> onChanged;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        SizedBox(width: 48, child: Text(label)),
        Expanded(
          child: Slider(value: value, min: min, max: max, onChanged: onChanged),
        ),
        SizedBox(
          width: 42,
          child: Text(value.toStringAsFixed(2), textAlign: TextAlign.end),
        ),
      ],
    );
  }
}

class _ChoiceSetting extends StatelessWidget {
  const _ChoiceSetting({
    required this.title,
    required this.value,
    required this.labels,
    required this.enabled,
    required this.onChanged,
  });

  final String title;
  final String? value;
  final Map<String, String> labels;
  final bool enabled;
  final ValueChanged<String> onChanged;

  @override
  Widget build(BuildContext context) {
    return ListTile(
      enabled: enabled,
      title: Text(title),
      trailing: DropdownButtonHideUnderline(
        child: DropdownButton<String>(
          value: labels.containsKey(value) ? value : null,
          hint: const Text('--'),
          items: labels.entries
              .map(
                (item) =>
                    DropdownMenuItem(value: item.key, child: Text(item.value)),
              )
              .toList(),
          onChanged: enabled
              ? (value) {
                  if (value != null) onChanged(value);
                }
              : null,
        ),
      ),
    );
  }
}

class _SwitchSetting extends StatelessWidget {
  const _SwitchSetting({
    required this.title,
    required this.subtitle,
    required this.value,
    required this.enabled,
    required this.onChanged,
  });

  final String title;
  final String subtitle;
  final bool value;
  final bool enabled;
  final ValueChanged<bool> onChanged;

  @override
  Widget build(BuildContext context) {
    return SwitchListTile(
      title: Text(title),
      subtitle: Text(
        subtitle,
        style: const TextStyle(fontSize: 12, color: Colors.white38),
      ),
      value: value,
      onChanged: enabled ? onChanged : null,
    );
  }
}

class _PageHeader extends StatelessWidget {
  const _PageHeader({
    required this.camera,
    required this.title,
    required this.subtitle,
  });

  final YiCameraController camera;
  final String title;
  final String subtitle;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(18, 16, 14, 10),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: const TextStyle(
                    fontSize: 24,
                    fontWeight: FontWeight.w500,
                  ),
                ),
                Text(
                  subtitle,
                  style: const TextStyle(
                    color: Colors.white38,
                    fontSize: 11,
                    letterSpacing: .5,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _Pill extends StatelessWidget {
  const _Pill({
    required this.icon,
    required this.label,
    this.color = Colors.white,
  });

  final IconData icon;
  final String label;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: BoxDecoration(
        color: const Color(0xAA090A0C),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, color: color, size: 15),
            const SizedBox(width: 5),
            Text(
              label,
              style: const TextStyle(
                fontWeight: FontWeight.w800,
                fontSize: 10,
                letterSpacing: .5,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _RoundButton extends StatelessWidget {
  const _RoundButton({
    required this.icon,
    required this.label,
    required this.onTap,
  });

  final IconData icon;
  final String label;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return InkWell(
      borderRadius: BorderRadius.circular(18),
      onTap: onTap,
      child: Padding(
        padding: const EdgeInsets.all(6),
        child: Column(
          children: [
            Container(
              width: 44,
              height: 44,
              decoration: const BoxDecoration(
                color: _surfaceHigh,
                shape: BoxShape.circle,
              ),
              child: Icon(icon, size: 21),
            ),
            const SizedBox(height: 5),
            Text(
              label,
              style: const TextStyle(color: Colors.white54, fontSize: 11),
            ),
          ],
        ),
      ),
    );
  }
}

class _MessageBar extends StatelessWidget {
  const _MessageBar({
    required this.message,
    required this.isError,
    required this.onClose,
  });

  final String message;
  final bool isError;
  final VoidCallback onClose;

  @override
  Widget build(BuildContext context) {
    final color = isError ? Colors.orangeAccent : const Color(0xFF56E39F);
    return Container(
      margin: const EdgeInsets.fromLTRB(12, 4, 12, 8),
      padding: const EdgeInsets.fromLTRB(13, 9, 5, 9),
      decoration: BoxDecoration(
        color: color.withValues(alpha: .12),
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: color.withValues(alpha: .25)),
      ),
      child: Row(
        children: [
          Icon(
            isError ? Icons.info_outline : Icons.check_circle_outline,
            color: color,
            size: 19,
          ),
          const SizedBox(width: 9),
          Expanded(
            child: Text(message, style: TextStyle(color: color, fontSize: 12)),
          ),
          IconButton(
            onPressed: onClose,
            icon: const Icon(Icons.close, size: 17),
            visualDensity: VisualDensity.compact,
          ),
        ],
      ),
    );
  }
}

class _EmptyState extends StatelessWidget {
  const _EmptyState({
    required this.icon,
    required this.title,
    required this.subtitle,
    this.action,
  });

  final IconData icon;
  final String title;
  final String subtitle;
  final Widget? action;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(28),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 52, color: Colors.white24),
            const SizedBox(height: 16),
            Text(
              title,
              style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w800),
            ),
            const SizedBox(height: 7),
            Text(
              subtitle,
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.white38, height: 1.45),
            ),
            if (action != null) ...[const SizedBox(height: 18), action!],
          ],
        ),
      ),
    );
  }
}

const _resolutions = <String, String>{
  'v_3840x2160_30p_16x9': '4K · 30 fps',
  'v_1920x1080_120p_16x9': '1080p · 120 fps',
  'v_1920x1080_60p_16x9': '1080p · 60 fps',
  'v_1920x1080_30p_16x9': '1080p · 30 fps',
  'v_1280x720_240p_16x9': '720p · 240 fps',
  'v_1280x720_60p_16x9_super': '720p · 60 fps',
};

const _filmFilters = <String, String>{
  'positive': '正片',
  'negative': '負片',
  'vivid': '鮮豔',
  'leicaAuthentic': '徠卡經典',
  'leicaVibrant': '徠卡鮮豔',
  'bloom': '繁花如夢',
  'blackCrystal': '黑晶',
  'redGlow': '赤霞紅',
  'mountainCyan': '遠山青',
  'lively': '生動',
  'custom': '我的濾鏡',
};

String _filmLabel(String value) => _filmFilters[value] ?? '負片';

List<double> _filmMatrix(String filter) => switch (filter) {
  'positive' => const [
    1.07,
    -.02,
    -.01,
    0,
    -3,
    -.01,
    1.06,
    -.01,
    0,
    -2,
    -.02,
    -.01,
    1.04,
    0,
    -2,
    0,
    0,
    0,
    1,
    0,
  ],
  'negative' => const [
    1.03,
    .01,
    -.03,
    0,
    3,
    -.02,
    1.01,
    .02,
    0,
    2,
    -.03,
    .04,
    .96,
    0,
    3,
    0,
    0,
    0,
    1,
    0,
  ],
  'vivid' => const [
    1.18,
    -.06,
    -.04,
    0,
    -3,
    -.04,
    1.14,
    -.02,
    0,
    -2,
    -.05,
    -.03,
    1.16,
    0,
    -2,
    0,
    0,
    0,
    1,
    0,
  ],
  'leicaAuthentic' => const [
    1.08,
    -.02,
    -.02,
    0,
    -9,
    -.03,
    1.06,
    .01,
    0,
    -7,
    -.02,
    .01,
    1.02,
    0,
    -6,
    0,
    0,
    0,
    1,
    0,
  ],
  'leicaVibrant' => const [
    1.14,
    -.04,
    -.03,
    0,
    -5,
    -.03,
    1.11,
    -.01,
    0,
    -3,
    -.04,
    -.01,
    1.10,
    0,
    -3,
    0,
    0,
    0,
    1,
    0,
  ],
  'bloom' => const [
    1.11,
    .02,
    -.04,
    0,
    7,
    .01,
    1.06,
    -.02,
    0,
    5,
    .01,
    .01,
    1.02,
    0,
    8,
    0,
    0,
    0,
    1,
    0,
  ],
  'blackCrystal' => const [
    1.12,
    -.03,
    -.03,
    0,
    -14,
    -.02,
    1.10,
    -.02,
    0,
    -13,
    -.02,
    -.02,
    1.12,
    0,
    -12,
    0,
    0,
    0,
    1,
    0,
  ],
  'redGlow' => const [
    1.15,
    .02,
    -.05,
    0,
    5,
    .01,
    1.02,
    -.02,
    0,
    1,
    -.04,
    .02,
    .94,
    0,
    0,
    0,
    0,
    0,
    1,
    0,
  ],
  'mountainCyan' => const [
    .96,
    .01,
    .02,
    0,
    0,
    -.02,
    1.08,
    .01,
    0,
    2,
    -.02,
    .04,
    1.10,
    0,
    3,
    0,
    0,
    0,
    1,
    0,
  ],
  'lively' => const [
    1.10,
    -.03,
    -.02,
    0,
    2,
    -.02,
    1.10,
    -.02,
    0,
    2,
    -.03,
    -.02,
    1.11,
    0,
    2,
    0,
    0,
    0,
    1,
    0,
  ],
  'classic' => const [
    1.08,
    -0.03,
    -0.02,
    0,
    4,
    -0.02,
    1.03,
    -0.01,
    0,
    2,
    -0.04,
    0.02,
    0.92,
    0,
    -2,
    0,
    0,
    0,
    1,
    0,
  ],
  'gold' => const [
    1.12,
    0.02,
    -0.04,
    0,
    5,
    0.01,
    1.02,
    -0.03,
    0,
    2,
    -0.06,
    0.02,
    0.88,
    0,
    -4,
    0,
    0,
    0,
    1,
    0,
  ],
  'cine' => const [
    0.95,
    0.03,
    0.02,
    0,
    -2,
    -0.02,
    1.06,
    0.01,
    0,
    1,
    0.01,
    0.04,
    1.08,
    0,
    1,
    0,
    0,
    0,
    1,
    0,
  ],
  'mono' => const [
    .213,
    .715,
    .072,
    0,
    0,
    .213,
    .715,
    .072,
    0,
    0,
    .213,
    .715,
    .072,
    0,
    0,
    0,
    0,
    0,
    1,
    0,
  ],
  _ => const [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
};

List<double> _currentFilmMatrix(YiCameraController camera) {
  if (camera.filmFilter != 'custom') return _filmMatrix(camera.filmFilter);
  final saturation = camera.customSaturation;
  final inverse = 1 - saturation;
  final contrast = camera.customContrast * (1 - camera.customFade * .25);
  final offset = (1 - contrast) * 127.5 + camera.customFade * 14;
  final warmth = camera.customWarmth;
  return [
    (inverse * .213 + saturation) * contrast + warmth * .08,
    inverse * .715 * contrast,
    inverse * .072 * contrast - warmth * .03,
    0,
    offset + warmth * 5,
    inverse * .213 * contrast,
    (inverse * .715 + saturation) * contrast,
    inverse * .072 * contrast,
    0,
    offset,
    inverse * .213 * contrast - warmth * .03,
    inverse * .715 * contrast,
    (inverse * .072 + saturation) * contrast - warmth * .07,
    0,
    offset - warmth * 5,
    0,
    0,
    0,
    1,
    0,
  ];
}

double _currentVignette(YiCameraController camera) =>
    switch (camera.filmFilter) {
      'leicaAuthentic' => .18,
      'blackCrystal' => .22,
      'redGlow' => .10,
      'custom' => camera.customVignette,
      _ => .04,
    };

String _resolutionLabel(String? value) => _resolutions[value] ?? '解析度';

String _labelFor(Object? value, Map<String, String> labels) =>
    labels[value?.toString()] ?? '設定';

String _fileSize(int bytes) {
  if (bytes >= 1073741824) {
    return '${(bytes / 1073741824).toStringAsFixed(1)} GB';
  }
  if (bytes >= 1048576) {
    return '${(bytes / 1048576).toStringAsFixed(1)} MB';
  }
  return '${(bytes / 1024).toStringAsFixed(0)} KB';
}

Future<void> _showResolutionPicker(
  BuildContext context,
  YiCameraController camera,
) => _showChoicePicker(
  context,
  camera: camera,
  title: '解析度與幀率',
  method: 'setVideoResolution',
  keyName: 'videoResolution',
  choices: _resolutions,
);

Future<void> _showChoicePicker(
  BuildContext context, {
  required YiCameraController camera,
  required String title,
  required String method,
  required String keyName,
  required Map<String, String> choices,
}) async {
  final selected = await showModalBottomSheet<String>(
    context: context,
    showDragHandle: true,
    builder: (context) => SafeArea(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(16, 0, 16, 20),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(8, 4, 8, 10),
              child: Text(
                title,
                style: const TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.w900,
                ),
              ),
            ),
            for (final item in choices.entries)
              ListTile(
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(14),
                ),
                title: Text(item.value),
                trailing: camera.settings[keyName] == item.key
                    ? const Icon(Icons.check_circle, color: _accent)
                    : null,
                onTap: () => Navigator.pop(context, item.key),
              ),
          ],
        ),
      ),
    ),
  );
  if (selected != null) {
    camera.updateSetting(method: method, key: keyName, value: selected);
  }
}
