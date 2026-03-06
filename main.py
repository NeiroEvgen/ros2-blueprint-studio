import sys
import os
from PySide6 import QtWidgets
from qt_material import apply_stylesheet

# Добавляем корень в путь
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from ui.launcher_dialog import LauncherDialog
from ui.main_window import RosVisualRunner

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    
    # Настройка темы
    extra = {
        'font_family': 'Segoe UI', 
        'font_size': '14px', 
        'line_height': '14px', 
        'density_scale': '-1'
    }
    apply_stylesheet(app, theme='dark_teal.xml', extra=extra)
    app.setStyleSheet(app.styleSheet() + """
        QLabel { color: #ffffff; font-weight: bold; }
        QPushButton { font-weight: bold; }
        QDockWidget { color: white; font-weight: bold; }
        QDockWidget::title { background: #2c3e50; text-align: center; padding: 5px; }
    """)

    # Лаунчер
    launcher = LauncherDialog()
    if launcher.exec() == QtWidgets.QDialog.Accepted:
        project_path = launcher.selected_project_path
        
        # Запуск главного окна
        window = RosVisualRunner(project_path)
        window.show()
        
        sys.exit(app.exec())
    else:
        sys.exit(0)