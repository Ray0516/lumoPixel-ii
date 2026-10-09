import 'camera_protocol.dart';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:gal/gal.dart';
import 'photo_processing.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter/foundation.dart';
import 'package:qr_flutter/qr_flutter.dart'; // 新增的 QR Code 套件

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // 1. 強制鎖定直立模式 (不允許橫置)
  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
    DeviceOrientation.portraitDown,
  ]);

  // 淺色介面使用深色狀態列圖示
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.dark,
      statusBarBrightness: Brightness.light,
    ),
  );

  runApp(const LumoPixelGoApp());
}

// 濾鏡列舉
enum PhotoFilter { pixel, normal, bw, sepia }

class LumoPixelGoApp extends StatelessWidget {
  const LumoPixelGoApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'LUMO PIXEL GO',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.light,
        scaffoldBackgroundColor: const Color(0xFFF5F6F2),
        primaryColor: const Color(0xFF202D29),
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF202D29),
          brightness: Brightness.light,
          primary: const Color(0xFF202D29),
          secondary: const Color(0xFFE1EDAB),
          surface: Colors.white,
        ),
        fontFamily: 'Roboto',
        textTheme: const TextTheme(
          bodyMedium: TextStyle(color: Color(0xFF202D29)),
          bodySmall: TextStyle(color: Color(0xFF737C76)),
        ),
        iconButtonTheme: IconButtonThemeData(
          style: IconButton.styleFrom(foregroundColor: const Color(0xFF202D29)),
        ),
        filledButtonTheme: FilledButtonThemeData(
          style: FilledButton.styleFrom(
            minimumSize: const Size(48, 48),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(14),
            ),
            textStyle: const TextStyle(
              fontWeight: FontWeight.w700,
              fontSize: 13,
            ),
          ),
        ),
        dividerTheme: const DividerThemeData(color: Color(0xFFE6E9E2)),
      ),
      home: const CameraDashboard(),
    );
  }
}

class CameraDashboard extends StatefulWidget {
  const CameraDashboard({super.key});

  @override
  State<CameraDashboard> createState() => _CameraDashboardState();
}

class _CameraDashboardState extends State<CameraDashboard> {
  static const String baseUrl = 'http://192.168.4.1';
  static const _ink = Color(0xFF202D29);
  static const _muted = Color(0xFF737C76);
  static const _line = Color(0xFFE6E9E2);
  static const _accent = Color(0xFFE1EDAB);

  PhotoFilter? get _activeGlobalFilter {
    final target = _selectionMode ? _selectedFiles.toList() : _fileList;
    if (target.isEmpty) return null;
    final first = _fileFilters[target.first] ?? PhotoFilter.pixel;
    return target.every(
          (file) => (_fileFilters[file] ?? PhotoFilter.pixel) == first,
        )
        ? first
        : null;
  }

  String _filterLabel(PhotoFilter filter) => switch (filter) {
    PhotoFilter.pixel => '像素',
    PhotoFilter.normal => '原圖',
    PhotoFilter.bw => '黑白',
    PhotoFilter.sepia => '復古',
  };

  bool _isLoading = false;
  bool _isConnected = false;
  bool _isDownloadingAll = false; // 追蹤是否正在一鍵下載
  String _downloadProgressText = '';
  String _storageInfo = 'Checking...';
  int _usagePercent = 0;
  List<String> _fileList = [];

  final Map<String, PhotoFilter> _fileFilters = {};
  bool _defaultPixelMode = true;
  int _pixelWidth = 120;
  double _bwContrast = 1.1;
  double _bwBrightness = 0;
  double _sepiaAmount = 0.65;
  int _jpegQuality = 95;
  bool _selectionMode = false;
  bool _cancelDownload = false;
  final Set<String> _selectedFiles = {};
  final Set<String> _downloadingFiles = {};
  Future<void> _saveSettingsFuture = Future<void>.value();

  @override
  void initState() {
    super.initState();
    _loadSettings();
  }

  Future<void> _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    if (!mounted) return;
    setState(() {
      _defaultPixelMode = prefs.getBool('defaultPixelMode') ?? true;
      _pixelWidth = (prefs.getInt('pixelWidth') ?? 120).clamp(24, 240);
      _bwContrast = (prefs.getDouble('bwContrast') ?? 1.1).clamp(0.5, 2.0);
      _bwBrightness = (prefs.getDouble('bwBrightness') ?? 0).clamp(-50.0, 50.0);
      _sepiaAmount = (prefs.getDouble('sepiaAmount') ?? 0.65).clamp(0.0, 1.0);
      final storedQuality = prefs.getInt('jpegQuality') ?? 95;
      _jpegQuality = [75, 85, 95].contains(storedQuality) ? storedQuality : 95;
    });
    await _fetchCameraData();
  }

  Future<void> _savePhotoSettings() async {
    final pixel = _pixelWidth;
    final contrast = _bwContrast;
    final brightness = _bwBrightness;
    final sepia = _sepiaAmount;
    final quality = _jpegQuality;
    final defaultPixel = _defaultPixelMode;
    _saveSettingsFuture = _saveSettingsFuture.then((_) async {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setBool('defaultPixelMode', defaultPixel);
      await prefs.setInt('pixelWidth', pixel);
      await prefs.setDouble('bwContrast', contrast);
      await prefs.setDouble('bwBrightness', brightness);
      await prefs.setDouble('sepiaAmount', sepia);
      await prefs.setInt('jpegQuality', quality);
    });
    await _saveSettingsFuture;
  }

  Future<void> _toggleDefaultPixelMode(bool value) async {
    if (!mounted) return;
    setState(() => _defaultPixelMode = value);
    await _savePhotoSettings();
  }

  Uri _photoUri(String endpoint, String filename) => Uri.parse(
    '$baseUrl/$endpoint',
  ).replace(queryParameters: {'file': filename});

  String _filterSummary(PhotoFilter filter) => switch (filter) {
    PhotoFilter.pixel => '像素密度 $_pixelWidth',
    PhotoFilter.bw =>
      '對比 ${_bwContrast.toStringAsFixed(1)}× · 亮度 ${_bwBrightness.round()}',
    PhotoFilter.sepia => '色調強度 ${(_sepiaAmount * 100).round()}%',
    PhotoFilter.normal => '保留原始檔案',
  };

  void _applyGlobalFilter(PhotoFilter filter) {
    final target = _selectionMode ? _selectedFiles.toList() : _fileList;
    if (target.isEmpty) return;
    setState(() {
      for (final file in target) {
        _fileFilters[file] = filter;
      }
    });
    _showSnackBar('已將 ${target.length} 張照片套用${_filterLabel(filter)}濾鏡');
  }

  void _toggleSelection(String filename) {
    setState(() {
      _selectionMode = true;
      if (!_selectedFiles.add(filename)) _selectedFiles.remove(filename);
    });
  }

  void _exitSelection() {
    setState(() {
      _selectionMode = false;
      _selectedFiles.clear();
    });
  }

  Future<void> _fetchCameraData() async {
    if (!mounted || _isLoading || _isDownloadingAll) return;
    setState(() => _isLoading = true);
    try {
      final statusRes = await http
          .get(Uri.parse('$baseUrl/status'))
          .timeout(const Duration(seconds: 5));
      final listRes = await http
          .get(Uri.parse('$baseUrl/list'))
          .timeout(const Duration(seconds: 5));
      if (statusRes.statusCode != 200 || listRes.statusCode != 200) {
        throw const FormatException('Camera response');
      }
      final status = json.decode(statusRes.body) as Map<String, dynamic>;
      final files = cameraPhotoFilenames(
        json.decode(listRes.body) as List<dynamic>,
      );
      if (!mounted) return;
      setState(() {
        _storageInfo = status['storage']?.toString() ?? '容量資訊未提供';
        _usagePercent = (status['usage'] as num? ?? 0).round().clamp(0, 100);
        _isConnected = true;
        _fileList = files;
        _selectedFiles.retainAll(files);
        _fileFilters.removeWhere((file, _) => !files.contains(file));
        for (final file in files) {
          _fileFilters.putIfAbsent(
            file,
            () => _defaultPixelMode ? PhotoFilter.pixel : PhotoFilter.normal,
          );
        }
      });
    } catch (_) {
      if (!mounted) return;
      setState(() => _isConnected = false);
      _showSnackBar('無法連線，請確認相機與 Wi-Fi', isError: true);
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  Future<bool> _ensureGalleryAccess() async {
    try {
      final allowed = await Gal.hasAccess() || await Gal.requestAccess();
      if (!allowed) _showSnackBar('請允許存取手機相簿，才能儲存照片', isError: true);
      return allowed;
    } catch (_) {
      _showSnackBar('此裝置無法存入相簿，請在手機 App 中下載', isError: true);
      return false;
    }
  }

  Future<bool> _downloadImage(
    String filename, {
    bool quiet = false,
    bool accessChecked = false,
  }) async {
    if (!mounted || _downloadingFiles.contains(filename)) return false;
    setState(() => _downloadingFiles.add(filename));
    try {
      if (!accessChecked && !await _ensureGalleryAccess()) return false;
      if (!mounted) return false;
      final filter =
          _fileFilters[filename] ??
          (_defaultPixelMode ? PhotoFilter.pixel : PhotoFilter.normal);
      final parameters = <String, dynamic>{
        'filter': filter.name,
        'pixelWidth': _pixelWidth,
        'bwContrast': _bwContrast,
        'bwBrightness': _bwBrightness,
        'sepiaAmount': _sepiaAmount,
        'jpegQuality': _jpegQuality,
      };
      final response = await http
          .get(_photoUri('view', filename))
          .timeout(const Duration(seconds: 15));
      if (response.statusCode != 200 || response.bodyBytes.isEmpty) {
        throw const FormatException('Photo unavailable');
      }
      final bytes = filter == PhotoFilter.normal
          ? response.bodyBytes
          : await compute(processPhotoExport, {
              ...parameters,
              'bytes': response.bodyBytes,
            });
      await Gal.putImageBytes(
        bytes,
        name:
            '${filename.replaceFirst(RegExp(r'\.[^.]+$'), '')}_${filter.name}',
      );
      if (!quiet) _showSnackBar('已儲存到手機相簿');
      return true;
    } catch (_) {
      if (!quiet) _showSnackBar('下載失敗，請確認相機連線後重試', isError: true);
      return false;
    } finally {
      if (mounted) setState(() => _downloadingFiles.remove(filename));
    }
  }

  Future<void> _downloadAllImages() => _downloadImages(
    _selectionMode
        ? _fileList.where(_selectedFiles.contains).toList()
        : List<String>.of(_fileList),
  );

  Future<void> _downloadImages(List<String> files) async {
    if (files.isEmpty || _isDownloadingAll || _downloadingFiles.isNotEmpty) {
      return;
    }
    setState(() {
      _isDownloadingAll = true;
      _cancelDownload = false;
      _downloadProgressText = '0 / ${files.length}';
    });
    final failed = <String>[];
    var completed = 0;
    var successes = 0;
    try {
      if (!await _ensureGalleryAccess()) return;
      for (final filename in files) {
        if (!mounted || _cancelDownload) break;
        final ok = await _downloadImage(
          filename,
          quiet: true,
          accessChecked: true,
        );
        completed++;
        if (ok) {
          successes++;
        } else {
          failed.add(filename);
        }
        if (!mounted) return;
        setState(() => _downloadProgressText = '$completed / ${files.length}');
        if (completed < files.length && !_cancelDownload) {
          await Future<void>.delayed(const Duration(milliseconds: 500));
        }
      }
    } finally {
      if (mounted) setState(() => _isDownloadingAll = false);
    }
    if (!mounted) return;
    final stopped = _cancelDownload ? '已停止，' : '';
    ScaffoldMessenger.of(context).clearSnackBars();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          '$stopped已儲存 $successes 張${failed.isEmpty ? '' : '，${failed.length} 張失敗'}',
        ),
        backgroundColor: _ink,
        action: failed.isEmpty
            ? null
            : SnackBarAction(
                label: '重試失敗',
                textColor: _accent,
                onPressed: () => _downloadImages(failed),
              ),
      ),
    );
  }

  Future<void> _deleteImage(String filename) async {
    final confirm = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        backgroundColor: Colors.white,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(24)),
        title: const Text(
          '刪除這張照片？',
          style: TextStyle(color: _ink, fontWeight: FontWeight.w700),
        ),
        content: Text(
          '將永久刪除相機中的 $filename。',
          style: const TextStyle(color: _muted),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('保留'),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text('刪除', style: TextStyle(color: Color(0xFFB14B42))),
          ),
        ],
      ),
    );

    if (confirm != true) return;

    try {
      final response = await http
          .get(_photoUri('delete', filename))
          .timeout(const Duration(seconds: 10));
      if (response.statusCode == 200) {
        _showSnackBar('檔案已刪除');
        _fetchCameraData();
      }
    } catch (e) {
      _showSnackBar('刪除失敗', isError: true);
    }
  }

  void _cycleFilter(String filename) {
    setState(() {
      PhotoFilter current = _fileFilters[filename] ?? PhotoFilter.pixel;
      int nextIndex = (current.index + 1) % PhotoFilter.values.length;
      _fileFilters[filename] = PhotoFilter.values[nextIndex];
    });
  }

  void _showQRDialog(String filename) {
    showDialog(
      context: context,
      builder: (context) => Dialog(
        backgroundColor: Colors.white,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(28)),
        child: Padding(
          padding: const EdgeInsets.all(28),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Icon(Icons.qr_code_2_rounded, size: 28, color: _ink),
              const SizedBox(height: 16),
              const Text(
                '分享照片',
                style: TextStyle(
                  fontSize: 23,
                  fontWeight: FontWeight.w700,
                  color: _ink,
                ),
              ),
              const SizedBox(height: 8),
              const Text(
                '先連上 Lumopixel_AP，再掃描查看照片。',
                textAlign: TextAlign.center,
                style: TextStyle(fontSize: 12, color: _muted, height: 1.6),
              ),
              const SizedBox(height: 24),
              SizedBox(
                width: 192,
                height: 192,
                child: QrImageView(
                  data: _photoUri('view', filename).toString(),
                  backgroundColor: Colors.white,
                ),
              ),
              const SizedBox(height: 16),
              Text(
                filename,
                style: const TextStyle(fontSize: 12, color: _muted),
              ),
              const SizedBox(height: 24),
              SizedBox(
                width: double.infinity,
                child: FilledButton(
                  onPressed: () => Navigator.pop(context),
                  child: const Text('完成'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Future<void> _showSettings() async {
    final filter = await showModalBottomSheet<PhotoFilter>(
      context: context,
      isScrollControlled: true,
      showDragHandle: true,
      backgroundColor: Colors.white,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(28)),
      ),
      builder: (sheetContext) => StatefulBuilder(
        builder: (context, setModalState) => SafeArea(
          top: false,
          child: ConstrainedBox(
            constraints: BoxConstraints(
              maxHeight: MediaQuery.sizeOf(context).height * 0.8,
            ),
            child: SingleChildScrollView(
              padding: const EdgeInsets.fromLTRB(24, 0, 24, 24),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    '設定',
                    style: TextStyle(fontSize: 24, fontWeight: FontWeight.w700),
                  ),
                  const SizedBox(height: 20),
                  SwitchListTile(
                    contentPadding: EdgeInsets.zero,
                    title: const Text(
                      '新照片預設使用像素濾鏡',
                      style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                    subtitle: const Text(
                      '關閉時使用原圖，不影響已載入照片',
                      style: TextStyle(fontSize: 11, color: _muted),
                    ),
                    activeThumbColor: _ink,
                    activeTrackColor: _accent,
                    value: _defaultPixelMode,
                    onChanged: (value) {
                      setModalState(() => _defaultPixelMode = value);
                      _toggleDefaultPixelMode(value);
                    },
                  ),
                  const Divider(height: 28),
                  Row(
                    children: [
                      const Expanded(
                        child: Text(
                          '濾鏡照片下載畫質',
                          style: TextStyle(
                            fontSize: 13,
                            fontWeight: FontWeight.w600,
                          ),
                        ),
                      ),
                      DropdownButton<int>(
                        key: const ValueKey('export-quality'),
                        value: _jpegQuality,
                        items: const [
                          DropdownMenuItem(value: 95, child: Text('高畫質 95%')),
                          DropdownMenuItem(value: 85, child: Text('標準 85%')),
                          DropdownMenuItem(value: 75, child: Text('省空間 75%')),
                        ],
                        style: const TextStyle(fontSize: 12, color: _ink),
                        underline: const SizedBox.shrink(),
                        onChanged: (value) {
                          if (value == null) return;
                          setState(() => _jpegQuality = value);
                          setModalState(() {});
                          _savePhotoSettings();
                        },
                      ),
                    ],
                  ),
                  const Text(
                    '原圖下載保留原始檔案，不重新壓縮。',
                    style: TextStyle(fontSize: 11, color: _muted),
                  ),
                  const SizedBox(height: 24),
                  const Text(
                    '濾鏡參數',
                    style: TextStyle(fontSize: 16, fontWeight: FontWeight.w700),
                  ),
                  const SizedBox(height: 8),
                  ...[PhotoFilter.pixel, PhotoFilter.bw, PhotoFilter.sepia].map(
                    (filter) => ListTile(
                      contentPadding: EdgeInsets.zero,
                      title: Text(
                        _filterLabel(filter),
                        style: const TextStyle(
                          fontSize: 13,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                      subtitle: Text(
                        _filterSummary(filter),
                        style: const TextStyle(fontSize: 11, color: _muted),
                      ),
                      trailing: const Icon(
                        Icons.chevron_right_rounded,
                        size: 20,
                      ),
                      onTap: () => Navigator.pop(sheetContext, filter),
                    ),
                  ),
                  const SizedBox(height: 12),
                  SizedBox(
                    width: double.infinity,
                    child: FilledButton(
                      onPressed: () => Navigator.pop(sheetContext),
                      child: const Text('完成'),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
    if (mounted && filter != null) await _showFilterSettings(filter);
  }

  Widget _parameterSlider({
    required String label,
    required String valueLabel,
    required double value,
    required double min,
    required double max,
    required int divisions,
    required ValueChanged<double> onChanged,
    required StateSetter setModalState,
  }) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Expanded(
              child: Text(
                label,
                style: const TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ),
            Text(
              valueLabel,
              style: const TextStyle(fontSize: 12, color: _muted),
            ),
          ],
        ),
        Slider(
          key: ValueKey(label),
          value: value,
          min: min,
          max: max,
          divisions: divisions,
          activeColor: _ink,
          inactiveColor: _line,
          onChanged: (value) {
            setState(() => onChanged(value));
            setModalState(() {});
          },
          onChangeEnd: (_) => _savePhotoSettings(),
        ),
      ],
    );
  }

  Future<void> _showFilterSettings(PhotoFilter initialFilter) async {
    var filter = initialFilter == PhotoFilter.normal
        ? PhotoFilter.pixel
        : initialFilter;
    final previewFile = _selectedFiles.isNotEmpty
        ? _selectedFiles.first
        : _fileList.firstOrNull;
    await showModalBottomSheet<void>(
      context: context,
      isScrollControlled: true,
      showDragHandle: true,
      backgroundColor: Colors.white,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(28)),
      ),
      builder: (sheetContext) => StatefulBuilder(
        builder: (context, setModalState) => SafeArea(
          top: false,
          child: ConstrainedBox(
            constraints: BoxConstraints(
              maxHeight: MediaQuery.sizeOf(context).height * 0.86,
            ),
            child: SingleChildScrollView(
              padding: const EdgeInsets.fromLTRB(24, 0, 24, 24),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    '濾鏡參數',
                    style: TextStyle(fontSize: 24, fontWeight: FontWeight.w700),
                  ),
                  const SizedBox(height: 16),
                  Row(
                    children:
                        [PhotoFilter.pixel, PhotoFilter.bw, PhotoFilter.sepia]
                            .map(
                              (item) => Expanded(
                                child: Padding(
                                  padding: const EdgeInsets.symmetric(
                                    horizontal: 3,
                                  ),
                                  child: TextButton(
                                    style: TextButton.styleFrom(
                                      backgroundColor: filter == item
                                          ? _ink
                                          : const Color(0xFFF5F6F2),
                                      foregroundColor: filter == item
                                          ? Colors.white
                                          : _ink,
                                    ),
                                    onPressed: () =>
                                        setModalState(() => filter = item),
                                    child: Text(_filterLabel(item)),
                                  ),
                                ),
                              ),
                            )
                            .toList(),
                  ),
                  const SizedBox(height: 16),
                  ClipRRect(
                    borderRadius: BorderRadius.circular(16),
                    child: SizedBox(
                      height: 175,
                      width: double.infinity,
                      child: previewFile == null
                          ? const ColoredBox(
                              color: Color(0xFFF5F6F2),
                              child: Center(
                                child: Text(
                                  '連線後可用相機照片預覽',
                                  style: TextStyle(fontSize: 12, color: _muted),
                                ),
                              ),
                            )
                          : _buildFilteredImagePreview(
                              _photoUri('view', previewFile).toString(),
                              filter,
                            ),
                    ),
                  ),
                  const SizedBox(height: 20),
                  if (filter == PhotoFilter.pixel) ...[
                    _parameterSlider(
                      label: '像素密度',
                      valueLabel: '$_pixelWidth',
                      value: _pixelWidth.toDouble(),
                      min: 24,
                      max: 240,
                      divisions: 18,
                      onChanged: (value) => _pixelWidth = value.round(),
                      setModalState: setModalState,
                    ),
                    const Text(
                      '數值越低，顆粒越大；數值越高，保留越多細節。',
                      style: TextStyle(fontSize: 11, color: _muted),
                    ),
                  ],
                  if (filter == PhotoFilter.bw) ...[
                    _parameterSlider(
                      label: '對比',
                      valueLabel: '${_bwContrast.toStringAsFixed(1)}×',
                      value: _bwContrast,
                      min: 0.5,
                      max: 2,
                      divisions: 15,
                      onChanged: (value) => _bwContrast = value,
                      setModalState: setModalState,
                    ),
                    _parameterSlider(
                      label: '亮度',
                      valueLabel: _bwBrightness.round().toString(),
                      value: _bwBrightness,
                      min: -50,
                      max: 50,
                      divisions: 20,
                      onChanged: (value) => _bwBrightness = value,
                      setModalState: setModalState,
                    ),
                  ],
                  if (filter == PhotoFilter.sepia) ...[
                    _parameterSlider(
                      label: '色調強度',
                      valueLabel: '${(_sepiaAmount * 100).round()}%',
                      value: _sepiaAmount,
                      min: 0,
                      max: 1,
                      divisions: 20,
                      onChanged: (value) => _sepiaAmount = value,
                      setModalState: setModalState,
                    ),
                    const Text(
                      '0% 保留原色，100% 使用完整復古色調。',
                      style: TextStyle(fontSize: 11, color: _muted),
                    ),
                  ],
                  const SizedBox(height: 20),
                  Row(
                    children: [
                      OutlinedButton(
                        onPressed: () {
                          setState(() {
                            if (filter == PhotoFilter.pixel) _pixelWidth = 120;
                            if (filter == PhotoFilter.bw) {
                              _bwContrast = 1.1;
                              _bwBrightness = 0;
                            }
                            if (filter == PhotoFilter.sepia) {
                              _sepiaAmount = 0.65;
                            }
                          });
                          setModalState(() {});
                          _savePhotoSettings();
                        },
                        child: const Text('重設'),
                      ),
                      const SizedBox(width: 12),
                      Expanded(
                        child: FilledButton(
                          onPressed:
                              (_selectionMode
                                  ? _selectedFiles.isEmpty
                                  : _fileList.isEmpty)
                              ? () => Navigator.pop(sheetContext)
                              : () {
                                  _applyGlobalFilter(filter);
                                  Navigator.pop(sheetContext);
                                },
                          child: Text(
                            _fileList.isEmpty
                                ? '完成'
                                : (_selectionMode ? '套用到選取照片' : '套用到全部照片'),
                          ),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
    await _savePhotoSettings();
  }

  Future<void> _showPhotoPreview(String filename) async {
    final files = List<String>.of(_fileList);
    var index = files.indexOf(filename);
    if (index < 0) return;
    final controller = PageController(initialPage: index);
    try {
      await showDialog<void>(
        context: context,
        builder: (dialogContext) => Dialog.fullscreen(
          child: StatefulBuilder(
            builder: (context, setModalState) {
              final current = files[index];
              final filter = _fileFilters[current] ?? PhotoFilter.pixel;
              return Scaffold(
                backgroundColor: const Color(0xFFF5F6F2),
                appBar: AppBar(
                  backgroundColor: const Color(0xFFF5F6F2),
                  leading: IconButton(
                    tooltip: '關閉預覽',
                    onPressed: () => Navigator.pop(dialogContext),
                    icon: const Icon(Icons.close_rounded),
                  ),
                  title: Text(current, style: const TextStyle(fontSize: 13)),
                  actions: [
                    Padding(
                      padding: const EdgeInsets.only(right: 20),
                      child: Center(
                        child: Text(
                          '${index + 1} / ${files.length}',
                          style: const TextStyle(fontSize: 12, color: _muted),
                        ),
                      ),
                    ),
                  ],
                ),
                body: SafeArea(
                  child: Column(
                    children: [
                      Expanded(
                        child: PageView.builder(
                          controller: controller,
                          itemCount: files.length,
                          onPageChanged: (value) =>
                              setModalState(() => index = value),
                          itemBuilder: (context, itemIndex) =>
                              InteractiveViewer(
                                minScale: 1,
                                maxScale: 5,
                                child: SizedBox.expand(
                                  child: _buildFilteredImagePreview(
                                    _photoUri(
                                      'view',
                                      files[itemIndex],
                                    ).toString(),
                                    _fileFilters[files[itemIndex]] ??
                                        PhotoFilter.pixel,
                                    fit: BoxFit.contain,
                                  ),
                                ),
                              ),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
                        child: Row(
                          children: PhotoFilter.values
                              .map(
                                (item) => Expanded(
                                  child: TextButton(
                                    onPressed: () {
                                      setState(
                                        () => _fileFilters[current] = item,
                                      );
                                      setModalState(() {});
                                    },
                                    style: TextButton.styleFrom(
                                      backgroundColor: item == filter
                                          ? _ink
                                          : Colors.transparent,
                                      foregroundColor: item == filter
                                          ? Colors.white
                                          : _muted,
                                    ),
                                    child: Text(_filterLabel(item)),
                                  ),
                                ),
                              )
                              .toList(),
                        ),
                      ),
                      Padding(
                        padding: const EdgeInsets.fromLTRB(24, 8, 24, 16),
                        child: Row(
                          children: [
                            Expanded(
                              child: FilledButton.icon(
                                onPressed: () => _downloadImage(current),
                                icon: const Icon(
                                  Icons.download_rounded,
                                  size: 18,
                                ),
                                label: const Text('下載照片'),
                              ),
                            ),
                            const SizedBox(width: 12),
                            IconButton(
                              tooltip: 'QR Code 分享',
                              onPressed: () => _showQRDialog(current),
                              icon: const Icon(Icons.qr_code_2_rounded),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      );
    } finally {
      controller.dispose();
    }
  }

  void _showSnackBar(String message, {bool isError = false}) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          message,
          style: const TextStyle(color: Colors.white, fontSize: 13),
        ),
        backgroundColor: isError ? const Color(0xFF8F4A40) : _ink,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
        duration: const Duration(seconds: 2),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          SafeArea(
            child: Center(
              child: ConstrainedBox(
                constraints: const BoxConstraints(maxWidth: 720),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _buildHeader(),
                    Expanded(
                      child: RefreshIndicator(
                        color: _ink,
                        onRefresh: _fetchCameraData,
                        child: CustomScrollView(
                          physics: const AlwaysScrollableScrollPhysics(),
                          slivers: [
                            SliverToBoxAdapter(child: _buildCollectionTitle()),
                            SliverToBoxAdapter(child: _buildStorageIndicator()),
                            if (_isConnected && _fileList.isNotEmpty)
                              SliverToBoxAdapter(child: _buildGlobalControls()),
                            if (_isLoading && _fileList.isEmpty)
                              const SliverFillRemaining(
                                hasScrollBody: false,
                                child: Center(
                                  child: CircularProgressIndicator(
                                    strokeWidth: 2,
                                  ),
                                ),
                              )
                            else if (!_isConnected)
                              SliverFillRemaining(
                                hasScrollBody: false,
                                child: _buildErrorState(),
                              )
                            else if (_fileList.isEmpty)
                              SliverFillRemaining(
                                hasScrollBody: false,
                                child: _buildEmptyState(),
                              )
                            else
                              _buildGallery(),
                          ],
                        ),
                      ),
                    ),
                    _buildConnectionFooter(),
                  ],
                ),
              ),
            ),
          ),
          if (_isDownloadingAll) _buildDownloadProgress(),
        ],
      ),
    );
  }

  Widget _buildHeader() {
    return Padding(
      padding: const EdgeInsets.fromLTRB(24, 16, 16, 10),
      child: Row(
        children: [
          Expanded(
            child: FittedBox(
              fit: BoxFit.scaleDown,
              alignment: Alignment.centerLeft,
              child: Row(
                children: [
                  Container(
                    width: 34,
                    height: 34,
                    decoration: BoxDecoration(
                      color: _ink,
                      borderRadius: BorderRadius.circular(11),
                    ),
                    child: const Icon(
                      Icons.camera_alt_outlined,
                      color: _accent,
                      size: 19,
                    ),
                  ),
                  const SizedBox(width: 10),
                  const Text(
                    'lumo',
                    style: TextStyle(
                      fontSize: 29,
                      fontWeight: FontWeight.w900,
                      letterSpacing: -1.7,
                      height: 1,
                    ),
                  ),
                  const SizedBox(width: 8),
                  Container(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 7,
                      vertical: 4,
                    ),
                    decoration: BoxDecoration(
                      color: _accent,
                      borderRadius: BorderRadius.circular(5),
                    ),
                    child: const Text(
                      'PIXEL GO',
                      style: TextStyle(
                        fontSize: 8,
                        fontWeight: FontWeight.w800,
                        letterSpacing: 0.8,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(width: 12),
          IconButton(
            tooltip: '設定',
            onPressed: _isDownloadingAll ? null : _showSettings,
            icon: const Icon(Icons.tune_rounded, size: 21),
          ),
        ],
      ),
    );
  }

  Widget _buildCollectionTitle() {
    return Padding(
      padding: const EdgeInsets.fromLTRB(24, 14, 24, 16),
      child: Row(
        children: [
          const Text(
            '照片',
            style: TextStyle(fontSize: 24, fontWeight: FontWeight.w700),
          ),
          const SizedBox(width: 10),
          if (_isConnected)
            Text(
              '${_fileList.length} 張',
              style: const TextStyle(fontSize: 12, color: _muted),
            ),
          const Spacer(),
          if (_isConnected && _fileList.isNotEmpty)
            TextButton(
              onPressed: _isDownloadingAll
                  ? null
                  : () {
                      if (_selectionMode) {
                        _exitSelection();
                      } else {
                        setState(() => _selectionMode = true);
                      }
                    },
              child: Text(_selectionMode ? '取消' : '選取'),
            ),
        ],
      ),
    );
  }

  Widget _buildStorageIndicator() {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 24),
      child: Container(
        padding: const EdgeInsets.all(18),
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(20),
          border: Border.all(color: _line),
        ),
        child: Column(
          children: [
            Row(
              children: [
                const Icon(Icons.sd_storage_outlined, size: 18, color: _ink),
                const SizedBox(width: 9),
                const Expanded(
                  child: Text(
                    '相機儲存空間',
                    style: TextStyle(fontSize: 12, fontWeight: FontWeight.w700),
                  ),
                ),
                Text(
                  _isConnected ? '$_usagePercent%' : '—',
                  style: const TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 13),
            ClipRRect(
              borderRadius: BorderRadius.circular(4),
              child: LinearProgressIndicator(
                value: _isConnected ? (_usagePercent / 100).clamp(0.0, 1.0) : 0,
                minHeight: 5,
                color: _ink,
                backgroundColor: const Color(0xFFEEF0E9),
              ),
            ),
            const SizedBox(height: 10),
            Row(
              children: [
                Expanded(
                  child: Text(
                    _isConnected
                        ? _storageInfo
                        : (_isLoading ? '正在讀取相機資訊…' : '連線後顯示容量'),
                    style: const TextStyle(fontSize: 10, color: _muted),
                  ),
                ),
                Text(
                  _isConnected ? 'SD CARD' : 'WAITING',
                  style: const TextStyle(
                    fontSize: 8,
                    letterSpacing: 1.3,
                    color: _muted,
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildGlobalControls() {
    final active = _activeGlobalFilter;
    return Padding(
      padding: const EdgeInsets.fromLTRB(24, 14, 24, 16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Expanded(
                child: _selectionMode
                    ? Text(
                        '已選 ${_selectedFiles.length} 張',
                        style: const TextStyle(
                          fontSize: 13,
                          fontWeight: FontWeight.w700,
                        ),
                      )
                    : const Text(
                        '濾鏡',
                        style: TextStyle(
                          fontSize: 14,
                          fontWeight: FontWeight.w700,
                        ),
                      ),
              ),
              if (_selectionMode)
                TextButton(
                  onPressed: () => setState(() {
                    if (_selectedFiles.length == _fileList.length) {
                      _selectedFiles.clear();
                    } else {
                      _selectedFiles.addAll(_fileList);
                    }
                  }),
                  child: Text(
                    _selectedFiles.length == _fileList.length ? '清除' : '全選',
                    style: const TextStyle(fontSize: 12),
                  ),
                ),
              FilledButton.icon(
                onPressed:
                    _isDownloadingAll ||
                        _downloadingFiles.isNotEmpty ||
                        (_selectionMode && _selectedFiles.isEmpty)
                    ? null
                    : _downloadAllImages,
                style: FilledButton.styleFrom(
                  minimumSize: const Size(0, 40),
                  padding: const EdgeInsets.symmetric(horizontal: 12),
                  backgroundColor: _accent,
                  foregroundColor: _ink,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
                icon: const Icon(Icons.download_rounded, size: 16),
                label: Text(
                  _selectionMode ? '下載選取' : '全部下載',
                  style: const TextStyle(fontSize: 12),
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          Container(
            padding: const EdgeInsets.all(4),
            decoration: BoxDecoration(
              color: const Color(0xFFE9ECE4),
              borderRadius: BorderRadius.circular(13),
            ),
            child: Row(
              children: PhotoFilter.values.map((filter) {
                final selected = active == filter;
                return Expanded(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 1),
                    child: TextButton(
                      onPressed:
                          _isDownloadingAll ||
                              (_selectionMode && _selectedFiles.isEmpty)
                          ? null
                          : () => _applyGlobalFilter(filter),
                      style: TextButton.styleFrom(
                        minimumSize: const Size(0, 40),
                        padding: EdgeInsets.zero,
                        foregroundColor: selected ? Colors.white : _muted,
                        backgroundColor: selected ? _ink : Colors.transparent,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(10),
                        ),
                        textStyle: TextStyle(
                          fontSize: 12,
                          fontWeight: selected
                              ? FontWeight.w700
                              : FontWeight.w500,
                        ),
                      ),
                      child: Text(_filterLabel(filter)),
                    ),
                  ),
                );
              }).toList(),
            ),
          ),
          const SizedBox(height: 4),
          Row(
            children: [
              Expanded(
                child: Text(
                  _selectionMode
                      ? '濾鏡僅套用到選取照片'
                      : (active == null
                            ? '部分照片使用不同濾鏡'
                            : _filterSummary(active)),
                  style: const TextStyle(fontSize: 10, color: _muted),
                ),
              ),
              if (active != PhotoFilter.normal)
                TextButton(
                  onPressed: _isDownloadingAll
                      ? null
                      : () => _showFilterSettings(active ?? PhotoFilter.pixel),
                  style: TextButton.styleFrom(
                    minimumSize: const Size(0, 40),
                    padding: const EdgeInsets.only(left: 12),
                  ),
                  child: const Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.tune_rounded, size: 14),
                      SizedBox(width: 4),
                      Text('參數', style: TextStyle(fontSize: 11)),
                    ],
                  ),
                ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildErrorState() {
    return Padding(
      padding: const EdgeInsets.fromLTRB(32, 40, 32, 40),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Container(
            width: 88,
            height: 88,
            decoration: BoxDecoration(
              color: _accent,
              borderRadius: BorderRadius.circular(28),
            ),
            child: const Icon(Icons.camera_alt_outlined, size: 38, color: _ink),
          ),
          const SizedBox(height: 26),
          const Text(
            '相機未連線',
            style: TextStyle(
              fontSize: 23,
              fontWeight: FontWeight.w700,
              letterSpacing: -0.5,
            ),
          ),
          const SizedBox(height: 12),
          const Text(
            '開啟相機，再到手機的 Wi-Fi 設定\n連線至 Lumopixel_AP。',
            textAlign: TextAlign.center,
            style: TextStyle(fontSize: 13, color: _muted, height: 1.8),
          ),
          const SizedBox(height: 26),
          FilledButton.icon(
            onPressed: _isLoading ? null : _fetchCameraData,
            icon: const Icon(Icons.sync_rounded, size: 17),
            label: const Text('重新連線'),
          ),
          const SizedBox(height: 13),
          const Text(
            '照片會留在相機中，下載後存入手機相簿。',
            textAlign: TextAlign.center,
            style: TextStyle(fontSize: 10, color: _muted),
          ),
        ],
      ),
    );
  }

  Widget _buildEmptyState() {
    return const Padding(
      padding: EdgeInsets.all(32),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(Icons.photo_library_outlined, size: 46, color: _muted),
          SizedBox(height: 20),
          Text(
            '相機中沒有照片',
            style: TextStyle(fontSize: 22, fontWeight: FontWeight.w700),
          ),
          SizedBox(height: 10),
          Text(
            '拍攝後，下拉重新整理照片列表。',
            textAlign: TextAlign.center,
            style: TextStyle(color: _muted, fontSize: 12, height: 1.7),
          ),
        ],
      ),
    );
  }

  Widget _buildGallery() {
    return SliverPadding(
      padding: const EdgeInsets.fromLTRB(24, 0, 24, 24),
      sliver: SliverLayoutBuilder(
        builder: (context, constraints) => SliverGrid(
          gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: constraints.crossAxisExtent >= 560 ? 3 : 2,
            mainAxisSpacing: 14,
            crossAxisSpacing: 12,
            childAspectRatio: 0.73,
          ),
          delegate: SliverChildBuilderDelegate((context, index) {
            final filename = _fileList[index];
            return _buildPhotoCard(
              filename,
              _photoUri('view', filename).toString(),
              _fileFilters[filename] ?? PhotoFilter.pixel,
            );
          }, childCount: _fileList.length),
        ),
      ),
    );
  }

  Widget _buildPhotoCard(String filename, String imageUrl, PhotoFilter filter) {
    final selected = _selectedFiles.contains(filename);
    final downloading = _downloadingFiles.contains(filename);
    return GestureDetector(
      key: ValueKey('photo-$filename'),
      onTap: () => _selectionMode
          ? _toggleSelection(filename)
          : _showPhotoPreview(filename),
      onLongPress: _isDownloadingAll ? null : () => _toggleSelection(filename),
      child: Container(
        clipBehavior: Clip.antiAlias,
        decoration: BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.circular(17),
          border: Border.all(
            color: selected ? _ink : _line,
            width: selected ? 2 : 1,
          ),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Expanded(
              child: Stack(
                fit: StackFit.expand,
                children: [
                  _buildFilteredImagePreview(imageUrl, filter),
                  if (!_selectionMode)
                    Positioned(
                      left: 8,
                      bottom: 8,
                      child: Tooltip(
                        message: '切換這張照片的濾鏡',
                        child: Material(
                          color: Colors.white.withValues(alpha: 0.93),
                          borderRadius: BorderRadius.circular(9),
                          child: InkWell(
                            borderRadius: BorderRadius.circular(9),
                            onTap: _isDownloadingAll
                                ? null
                                : () => _cycleFilter(filename),
                            child: Padding(
                              padding: const EdgeInsets.symmetric(
                                horizontal: 10,
                                vertical: 10,
                              ),
                              child: Row(
                                mainAxisSize: MainAxisSize.min,
                                children: [
                                  const Icon(
                                    Icons.auto_awesome_outlined,
                                    size: 12,
                                    color: _ink,
                                  ),
                                  const SizedBox(width: 5),
                                  Text(
                                    _filterLabel(filter),
                                    style: const TextStyle(
                                      fontSize: 10,
                                      fontWeight: FontWeight.w700,
                                      color: _ink,
                                    ),
                                  ),
                                ],
                              ),
                            ),
                          ),
                        ),
                      ),
                    ),
                  if (_selectionMode)
                    Positioned(
                      top: 8,
                      right: 8,
                      child: Container(
                        width: 28,
                        height: 28,
                        decoration: BoxDecoration(
                          shape: BoxShape.circle,
                          color: selected ? _ink : Colors.white,
                          border: Border.all(color: selected ? _ink : _line),
                        ),
                        child: selected
                            ? const Icon(
                                Icons.check_rounded,
                                color: _accent,
                                size: 19,
                              )
                            : null,
                      ),
                    ),
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(12, 11, 12, 0),
              child: Text(
                filename.replaceFirst(RegExp(r'\.[^.]+$'), ''),
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: const TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ),
            if (_selectionMode)
              SizedBox(
                height: 44,
                child: Center(
                  child: Text(
                    selected ? '已選取' : '點選照片',
                    style: const TextStyle(fontSize: 11, color: _muted),
                  ),
                ),
              )
            else
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 1),
                child: Row(
                  children: [
                    Expanded(
                      child: IconButton(
                        tooltip: '下載照片',
                        onPressed: _isDownloadingAll || downloading
                            ? null
                            : () => _downloadImage(filename),
                        icon: downloading
                            ? const SizedBox(
                                width: 18,
                                height: 18,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                ),
                              )
                            : const Icon(Icons.download_rounded, size: 18),
                      ),
                    ),
                    Expanded(
                      child: IconButton(
                        tooltip: 'QR Code 分享',
                        onPressed: _isDownloadingAll
                            ? null
                            : () => _showQRDialog(filename),
                        icon: const Icon(Icons.qr_code_2_rounded, size: 18),
                      ),
                    ),
                    Expanded(
                      child: IconButton(
                        tooltip: '刪除照片',
                        onPressed: _isDownloadingAll || downloading
                            ? null
                            : () => _deleteImage(filename),
                        icon: const Icon(
                          Icons.delete_outline_rounded,
                          size: 18,
                          color: _muted,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildConnectionFooter() {
    return Container(
      padding: const EdgeInsets.fromLTRB(24, 8, 16, 8),
      decoration: const BoxDecoration(
        color: Color(0xFFF5F6F2),
        border: Border(top: BorderSide(color: _line)),
      ),
      child: Row(
        children: [
          Container(
            width: 6,
            height: 6,
            decoration: BoxDecoration(
              color: _isConnected
                  ? const Color(0xFF60815A)
                  : const Color(0xFFAEB4AC),
              shape: BoxShape.circle,
            ),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              _isLoading
                  ? '正在同步…'
                  : (_isConnected ? 'Lumopixel_AP 已連線' : '等待相機連線'),
              style: const TextStyle(fontSize: 10, color: _muted),
            ),
          ),
          IconButton(
            tooltip: '重新整理',
            onPressed: _isLoading || _isDownloadingAll
                ? null
                : _fetchCameraData,
            icon: const Icon(Icons.sync_rounded, size: 20),
          ),
        ],
      ),
    );
  }

  Widget _buildDownloadProgress() {
    return Positioned.fill(
      child: ColoredBox(
        color: _ink.withValues(alpha: 0.55),
        child: Center(
          child: Container(
            width: 280,
            padding: const EdgeInsets.all(28),
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(26),
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const CircularProgressIndicator(strokeWidth: 2, color: _ink),
                const SizedBox(height: 24),
                const Text(
                  '下載照片',
                  style: TextStyle(fontSize: 19, fontWeight: FontWeight.w700),
                ),
                const SizedBox(height: 12),
                Text(
                  _downloadProgressText,
                  style: const TextStyle(
                    fontSize: 28,
                    fontWeight: FontWeight.w700,
                  ),
                ),
                const SizedBox(height: 12),
                const Text(
                  '儲存到手機相簿，請保持相機連線。',
                  textAlign: TextAlign.center,
                  style: TextStyle(color: _muted, fontSize: 11, height: 1.8),
                ),
                const SizedBox(height: 16),
                TextButton(
                  onPressed: _cancelDownload
                      ? null
                      : () => setState(() => _cancelDownload = true),
                  child: Text(_cancelDownload ? '目前這張完成後停止' : '停止下載'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildFilteredImagePreview(
    String url,
    PhotoFilter filter, {
    BoxFit fit = BoxFit.contain,
  }) {
    Widget image = Image.network(
      url,
      fit: fit,
      cacheWidth: filter == PhotoFilter.pixel ? _pixelWidth : null,
      filterQuality: filter == PhotoFilter.pixel
          ? FilterQuality.none
          : FilterQuality.low,
      loadingBuilder: (context, child, progress) => progress == null
          ? child
          : const ColoredBox(
              color: Color(0xFFE9ECE4),
              child: Center(
                child: SizedBox(
                  width: 18,
                  height: 18,
                  child: CircularProgressIndicator(strokeWidth: 2, color: _ink),
                ),
              ),
            ),
      errorBuilder: (context, error, stackTrace) => const ColoredBox(
        color: Color(0xFFE9ECE4),
        child: Center(
          child: Icon(
            Icons.image_not_supported_outlined,
            color: _muted,
            size: 26,
          ),
        ),
      ),
    );
    final matrix = photoColorMatrix(
      filter.name,
      contrast: _bwContrast,
      brightness: _bwBrightness,
      sepiaAmount: _sepiaAmount,
    );
    if (matrix != null) {
      image = ColorFiltered(
        colorFilter: ColorFilter.matrix(matrix),
        child: image,
      );
    }
    return image;
  }
}
