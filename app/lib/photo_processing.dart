import 'dart:typed_data';
import 'package:image/image.dart' as img;

List<double>? photoColorMatrix(
  String filter, {
  double contrast = 1.1,
  double brightness = 0,
  double sepiaAmount = 0.65,
}) {
  if (filter == 'bw') {
    final c = contrast.clamp(0.5, 2.0);
    final offset = 128 * (1 - c) + brightness.clamp(-50, 50);
    return [
      0.2126 * c,
      0.7152 * c,
      0.0722 * c,
      0,
      offset,
      0.2126 * c,
      0.7152 * c,
      0.0722 * c,
      0,
      offset,
      0.2126 * c,
      0.7152 * c,
      0.0722 * c,
      0,
      offset,
      0,
      0,
      0,
      1,
      0,
    ];
  }
  if (filter == 'sepia') {
    final a = sepiaAmount.clamp(0.0, 1.0);
    return [
      1 - a + 0.393 * a,
      0.769 * a,
      0.189 * a,
      0,
      0,
      0.349 * a,
      1 - a + 0.686 * a,
      0.168 * a,
      0,
      0,
      0.272 * a,
      0.534 * a,
      1 - a + 0.131 * a,
      0,
      0,
      0,
      0,
      0,
      1,
      0,
    ];
  }
  return null;
}

Uint8List processPhotoExport(Map<String, dynamic> parameters) {
  final bytes = parameters['bytes'] as Uint8List;
  final filter = parameters['filter'] as String;
  if (filter == 'normal') return bytes;
  final decoded = img.decodeImage(bytes);
  if (decoded == null) throw const FormatException('Invalid photo');
  var image = img.bakeOrientation(decoded);
  if (filter == 'pixel') {
    final originalWidth = image.width;
    final originalHeight = image.height;
    final detail = ((parameters['pixelWidth'] as num?) ?? 120)
        .round()
        .clamp(24, 240)
        .clamp(1, originalWidth);
    final small = img.copyResize(
      image,
      width: detail,
      interpolation: img.Interpolation.linear,
    );
    image = img.copyResize(
      small,
      width: originalWidth,
      height: originalHeight,
      interpolation: img.Interpolation.nearest,
    );
  } else {
    final matrix = photoColorMatrix(
      filter,
      contrast: ((parameters['bwContrast'] as num?) ?? 1.1).toDouble(),
      brightness: ((parameters['bwBrightness'] as num?) ?? 0).toDouble(),
      sepiaAmount: ((parameters['sepiaAmount'] as num?) ?? 0.65).toDouble(),
    );
    if (matrix != null) {
      for (final pixel in image) {
        final r = pixel.r.toDouble();
        final g = pixel.g.toDouble();
        final b = pixel.b.toDouble();
        pixel.setRgb(
          (matrix[0] * r + matrix[1] * g + matrix[2] * b + matrix[4])
              .round()
              .clamp(0, 255),
          (matrix[5] * r + matrix[6] * g + matrix[7] * b + matrix[9])
              .round()
              .clamp(0, 255),
          (matrix[10] * r + matrix[11] * g + matrix[12] * b + matrix[14])
              .round()
              .clamp(0, 255),
        );
      }
    }
  }
  final quality = ((parameters['jpegQuality'] as num?) ?? 95).round().clamp(
    70,
    100,
  );
  return Uint8List.fromList(img.encodeJpg(image, quality: quality));
}
