import traceback
import sys

# 1. КРИЧИМ В КОНСОЛЬ
print("\n" + "!"*60)
print("ВАЖНО: Кто-то импортирует устаревший 'nodes.library.logic'!")
print("Файл, который это делает, находится здесь:")
# Выводим стек вызовов, чтобы найти виновника
traceback.print_stack(limit=2) 
print("!"*60 + "\n")

# 2. ПРОБРАСЫВАЕМ ИМПОРТЫ (ЧТОБЫ НЕ ПАДАЛО)
# Мы делаем вид, что мы и есть тот самый модуль, перенаправляя запрос в новые файлы
try:
    from .py_logic import TimerNode, PrintNode
    from .cpp_logic import CppTimerNode, CppPrintNode
except ImportError as e:
    print(f"CRITICAL ERROR inside logic.py trap: {e}")