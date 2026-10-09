import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_application_1/main.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:image/image.dart' as img;
import 'package:flutter_application_1/photo_processing.dart';
import 'package:flutter_application_1/camera_protocol.dart';

void main() {
  final savedNames = <String>[];
  setUp(() {
    SharedPreferences.setMockInitialValues({});
    savedNames.clear();
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockMethodCallHandler(const MethodChannel('gal'), (call) async {
          if (call.method == 'hasAccess' || call.method == 'requestAccess') {
            return true;
          }
          if (call.method == 'putImageBytes') {
            savedNames.add((call.arguments as Map)['name'] as String);
          }
          return null;
        });
  });

  Future<void> withCamera(
    Future<void> Function() body, {
    bool connected = true,
    List<String> files = const ['PHOTO_001.jpg', 'PHOTO_002.jpg'],
  }) {
    return http.runWithClient(
      body,
      () => MockClient((request) async {
        if (!connected) throw Exception('Camera offline');
        if (request.url.path == '/status') {
          return http.Response(
            jsonEncode({'storage': '4.2 MB / 16 MB', 'usage': 26}),
            200,
          );
        }
        if (request.url.path == '/list') {
          return http.Response(jsonEncode(files), 200);
        }
        if (request.url.path == '/view') {
          if (!cameraPhotoFilenames(
            files,
          ).contains(request.url.queryParameters['file'])) {
            return http.Response('', 404);
          }
          final image = img.Image(width: 48, height: 32);
          img.fill(image, color: img.ColorRgb8(150, 100, 70));
          return http.Response.bytes(img.encodeJpg(image), 200);
        }
        return http.Response('', 404);
      }),
    );
  }

  testWidgets('Offline camera presents connection steps and retry', (
    tester,
  ) async {
    await withCamera(() async {
      tester.view.physicalSize = const Size(390, 844);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      await tester.pumpWidget(const LumoPixelGoApp());
      await tester.pumpAndSettle();
      expect(find.text('相機未連線'), findsOneWidget);
      expect(find.text('重新連線'), findsOneWidget);
      expect(find.text('Checking...'), findsNothing);
      expect(tester.takeException(), isNull);
    }, connected: false);
  });

  testWidgets(
    'Small phone supports global filters, sharing and stored settings',
    (tester) async {
      await withCamera(() async {
        tester.view.physicalSize = const Size(320, 760);
        tester.view.devicePixelRatio = 1;
        addTearDown(tester.view.resetPhysicalSize);
        addTearDown(tester.view.resetDevicePixelRatio);
        await tester.pumpWidget(const LumoPixelGoApp());
        await tester.pumpAndSettle();
        expect(find.text('PHOTO_002'), findsOneWidget);
        expect(find.text('26%'), findsOneWidget);
        expect(tester.takeException(), isNull);
        await tester.tap(find.text('黑白'));
        await tester.pumpAndSettle();
        expect(find.text('黑白'), findsNWidgets(3));
        await tester.tap(find.byTooltip('QR Code 分享').first);
        await tester.pumpAndSettle();
        expect(find.text('分享照片'), findsOneWidget);
        expect(find.text('PHOTO_002.jpg'), findsOneWidget);
        expect(tester.takeException(), isNull);
        await tester.tap(find.text('完成'));
        await tester.pumpAndSettle();
        await tester.tap(find.byTooltip('設定'));
        await tester.pumpAndSettle();
        await tester.tap(find.byType(Switch));
        await tester.pumpAndSettle();
        final prefs = await SharedPreferences.getInstance();
        expect(prefs.getBool('defaultPixelMode'), isFalse);
        expect(tester.takeException(), isNull);
      });
    },
  );

  testWidgets('Connected empty camera shows an intentional empty state', (
    tester,
  ) async {
    await withCamera(() async {
      await tester.pumpWidget(const LumoPixelGoApp());
      await tester.pumpAndSettle();
      expect(find.text('相機中沒有照片'), findsOneWidget);
      expect(find.text('Lumopixel_AP 已連線'), findsOneWidget);
      expect(tester.takeException(), isNull);
    }, files: []);
  });

  testWidgets('Filter parameters persist and apply only to selected photos', (
    tester,
  ) async {
    await withCamera(() async {
      tester.view.physicalSize = const Size(390, 844);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      await tester.pumpWidget(const LumoPixelGoApp());
      await tester.pumpAndSettle();
      expect(find.text('THE LITTLE MOMENTS'), findsNothing);
      expect(find.text('把日常，收進一格。'), findsNothing);
      await tester.longPress(find.byKey(const ValueKey('photo-PHOTO_002.jpg')));
      await tester.pumpAndSettle();
      expect(find.text('已選 1 張'), findsOneWidget);
      await tester.tap(find.text('參數'));
      await tester.pumpAndSettle();
      final slider = find.byKey(const ValueKey('像素密度'));
      await tester.tapAt(tester.getTopLeft(slider) + const Offset(50, 24));
      await tester.pumpAndSettle();
      final prefs = await SharedPreferences.getInstance();
      expect(prefs.getInt('pixelWidth'), lessThan(120));
      await tester.tap(find.text('復古').last);
      await tester.pumpAndSettle();
      await tester.tap(find.text('套用到選取照片'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('取消'));
      await tester.pumpAndSettle();
      expect(find.text('部分照片使用不同濾鏡'), findsOneWidget);
      expect(find.text('復古'), findsNWidgets(2));
      expect(find.text('像素'), findsNWidgets(2));
      expect(tester.takeException(), isNull);
    });
  });

  testWidgets(
    'Selected download saves only selected photo and uses safe filename encoding',
    (tester) async {
      await withCamera(() async {
        tester.view.physicalSize = const Size(390, 844);
        tester.view.devicePixelRatio = 1;
        addTearDown(tester.view.resetPhysicalSize);
        addTearDown(tester.view.resetDevicePixelRatio);
        await tester.pumpWidget(const LumoPixelGoApp());
        await tester.pumpAndSettle();
        await tester.longPress(
          find.byKey(const ValueKey('photo-PHOTO & 002.jpg')),
        );
        await tester.pumpAndSettle();
        await tester.tap(find.text('原圖'));
        await tester.pumpAndSettle();
        await tester.tap(find.text('下載選取'));
        await tester.pumpAndSettle();
        await tester.pump(const Duration(milliseconds: 100));
        await tester.pumpAndSettle();
        expect(savedNames, ['PHOTO & 002_normal']);
        expect(find.text('已儲存 1 張'), findsOneWidget);
        expect(tester.takeException(), isNull);
      }, files: ['PHOTO_001.jpg', 'PHOTO & 002.jpg']);
    },
  );

  testWidgets('Photo opens full preview and allows individual filter changes', (
    tester,
  ) async {
    await withCamera(() async {
      tester.view.physicalSize = const Size(390, 844);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      await tester.pumpWidget(const LumoPixelGoApp());
      await tester.pumpAndSettle();
      await tester.tapAt(
        tester.getTopLeft(find.byKey(const ValueKey('photo-PHOTO_002.jpg'))) +
            const Offset(40, 40),
      );
      await tester.pumpAndSettle();
      expect(find.byType(InteractiveViewer), findsWidgets);
      expect(find.text('1 / 2'), findsOneWidget);
      await tester.tap(find.text('黑白').last);
      await tester.pumpAndSettle();
      await tester.tap(find.byTooltip('關閉預覽'));
      await tester.pumpAndSettle();
      expect(find.text('部分照片使用不同濾鏡'), findsOneWidget);
      expect(tester.takeException(), isNull);
    });
  });

  test(
    'ESP32 file paths become unique image basenames in newest-first order',
    () {
      expect(
        cameraPhotoFilenames([
          '/Lumopixel/IMG_100.jpg',
          'Lumopixel/IMG_200.JPG',
          'IMG_100.jpg',
          '/Lumopixel',
          'qrcode.min.js',
          null,
          42,
        ]),
        ['IMG_200.JPG', 'IMG_100.jpg'],
      );
    },
  );

  testWidgets(
    'Firmware path entries download using the basename expected by view',
    (tester) async {
      tester.view.physicalSize = const Size(390, 844);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      SharedPreferences.setMockInitialValues({'defaultPixelMode': false});
      await withCamera(() async {
        await tester.pumpWidget(const LumoPixelGoApp());
        await tester.pumpAndSettle();
        expect(find.text('IMG_100'), findsOneWidget);
        await tester.tap(find.byTooltip('下載照片').first);
        await tester.pumpAndSettle();
        expect(savedNames, ['IMG_100_normal']);
        expect(tester.takeException(), isNull);
      }, files: ['Lumopixel/IMG_100.jpg']);
    },
  );

  test('Export keeps original bytes and small image dimensions', () {
    final image = img.Image(width: 12, height: 8);
    img.fill(image, color: img.ColorRgb8(130, 90, 60));
    final bytes = img.encodePng(image);
    expect(
      processPhotoExport({'bytes': bytes, 'filter': 'normal'}),
      same(bytes),
    );
    final result = img.decodeImage(
      processPhotoExport({
        'bytes': bytes,
        'filter': 'pixel',
        'pixelWidth': 120,
      }),
    );
    expect(result!.width, 12);
    expect(result.height, 8);
  });

  testWidgets('Repeated taps do not duplicate a pending photo download', (
    tester,
  ) async {
    SharedPreferences.setMockInitialValues({'defaultPixelMode': false});
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockMethodCallHandler(const MethodChannel('gal'), (call) async {
          if (call.method == 'hasAccess') {
            await Future<void>.delayed(const Duration(milliseconds: 100));
            return true;
          }
          if (call.method == 'requestAccess') return true;
          if (call.method == 'putImageBytes') {
            savedNames.add((call.arguments as Map)['name'] as String);
          }
          return null;
        });
    await withCamera(() async {
      tester.view.physicalSize = const Size(390, 844);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      await tester.pumpWidget(const LumoPixelGoApp());
      await tester.pumpAndSettle();
      await tester.tap(find.byTooltip('下載照片').first);
      await tester.tap(find.byTooltip('下載照片').first);
      await tester.pump(const Duration(milliseconds: 150));
      await tester.pumpAndSettle();
      expect(savedNames, ['PHOTO_002_normal']);
      expect(tester.takeException(), isNull);
    });
  });

  testWidgets('Denied gallery access leaves download controls usable', (
    tester,
  ) async {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
        .setMockMethodCallHandler(
          const MethodChannel('gal'),
          (call) async => false,
        );
    await withCamera(() async {
      await tester.pumpWidget(const LumoPixelGoApp());
      await tester.pumpAndSettle();
      await tester.tap(find.text('全部下載'));
      await tester.pumpAndSettle();
      expect(savedNames, isEmpty);
      expect(find.text('停止下載'), findsNothing);
      expect(find.text('請允許存取手機相簿，才能儲存照片'), findsOneWidget);
      expect(tester.takeException(), isNull);
    });
  });

  test('Export respects black and white brightness and sepia strength', () {
    final image = img.Image(width: 16, height: 16);
    img.fill(image, color: img.ColorRgb8(120, 90, 60));
    final bytes = img.encodePng(image);
    final dark = img
        .decodeImage(
          processPhotoExport({
            'bytes': bytes,
            'filter': 'bw',
            'bwBrightness': -25,
            'jpegQuality': 95,
          }),
        )!
        .getPixel(8, 8);
    final bright = img
        .decodeImage(
          processPhotoExport({
            'bytes': bytes,
            'filter': 'bw',
            'bwBrightness': 25,
            'jpegQuality': 95,
          }),
        )!
        .getPixel(8, 8);
    expect(dark.r, closeTo(dark.g, 2));
    expect(dark.g, closeTo(dark.b, 2));
    expect(bright.r - dark.r, closeTo(50, 4));
    final noSepia = img
        .decodeImage(
          processPhotoExport({
            'bytes': bytes,
            'filter': 'sepia',
            'sepiaAmount': 0,
            'jpegQuality': 95,
          }),
        )!
        .getPixel(8, 8);
    expect(noSepia.r, closeTo(120, 3));
    final fullSepia = img
        .decodeImage(
          processPhotoExport({
            'bytes': bytes,
            'filter': 'sepia',
            'sepiaAmount': 1,
            'jpegQuality': 95,
          }),
        )!
        .getPixel(8, 8);
    expect(fullSepia.r, greaterThan(fullSepia.b));
    expect(fullSepia.g, greaterThan(noSepia.g));
  });
}
