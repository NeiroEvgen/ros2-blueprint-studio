import sys

print("--- НАЧАЛО ТЕСТА ---")

print("1. Проверка Python...")
print(f"   Версия: {sys.version}")

print("2. Импорт PySide6...")
try:
    from PySide6 import QtWidgets
    print("   PySide6 УСПЕШНО.")
except Exception as e:
    print(f"   !!! ОШИБКА PySide6: {e}")

print("3. Импорт NodeGraphQt...")
try:
    import NodeGraphQt
    print("   NodeGraphQt УСПЕШНО.")
except Exception as e:
    print(f"   !!! ОШИБКА NodeGraphQt: {e}")

print("4. Проверка GUI (должно появиться маленькое окно)...")
try:
    app = QtWidgets.QApplication(sys.argv)
    window = QtWidgets.QLabel("ЕСЛИ ВЫ ЭТО ЧИТАЕТЕ - ВСЕ ОК!\nЗакройте это окно.")
    window.resize(400, 200)
    window.show()
    print("   Окно создано, ждем закрытия...")
    app.exec()
    print("   Окно закрыто. Тест пройден.")
except Exception as e:
    print(f"   !!! ОШИБКА GUI: {e}")

print("--- КОНЕЦ ТЕСТА ---")
input("Нажмите Enter, чтобы выйти...")