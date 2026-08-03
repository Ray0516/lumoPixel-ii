import 'package:flutter_test/flutter_test.dart';
import 'package:yi_4k_remote/main.dart';

void main() {
  testWidgets('shows the YI camera connection flow', (tester) async {
    await tester.pumpWidget(const YiRemoteApp());

    expect(find.text('LUMO'), findsWidgets);
    expect(find.text('連接你的 YI 4K'), findsOneWidget);
    expect(find.text('連接相機'), findsOneWidget);
  });
}
