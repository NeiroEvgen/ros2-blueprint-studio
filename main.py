import sys
import os
from PySide6 import QtWidgets
from qt_material import apply_stylesheet

# add the project root to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from core import settings as _settings
_settings.apply_language()          # saved UI language, before any window is created

from ui.launcher_dialog import LauncherDialog
from ui.main_window import RosVisualRunner

from NodeGraphQt.qgraphics import node_base as _nb
_orig_asm = _nb.NodeItem.auto_switch_mode
def _safe_asm(self):
    try:
        if self.viewer() is None:
            return
        _orig_asm(self)
    except Exception:
        pass
_nb.NodeItem.auto_switch_mode = _safe_asm

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    
    # theme
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

    # launcher
    launcher = LauncherDialog()
    if launcher.exec() == QtWidgets.QDialog.Accepted:
        project_path = launcher.selected_project_path
        
        # main window
        window = RosVisualRunner(project_path)
        window.show()
        
        sys.exit(app.exec())
    else:
        sys.exit(0)