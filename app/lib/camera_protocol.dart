/// ESP32 SD libraries can return either a filename or its directory path.
/// The camera's /view and /delete handlers expect only the filename.
List<String> cameraPhotoFilenames(List<dynamic> entries) {
  final filenames = <String>{};
  final imageExtension = RegExp(r'\.(jpe?g|png)$', caseSensitive: false);
  for (final entry in entries) {
    if (entry is! String) continue;
    final filename = entry.replaceAll('\\', '/').split('/').last;
    if (imageExtension.hasMatch(filename)) filenames.add(filename);
  }
  return filenames.toList().reversed.toList();
}
