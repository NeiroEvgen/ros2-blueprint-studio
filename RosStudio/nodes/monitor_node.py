from PySide6 import QtWidgets, QtCore, QtGui
from NodeGraphQt import BaseNode, NodeBaseWidget

# --- 1. Console Window ---
class NodeLogWindow(QtWidgets.QDialog):
    def __init__(self, node_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Log: {node_name}")
        self.resize(600, 400)
        
        layout = QtWidgets.QVBoxLayout(self)
        
        self.text_edit = QtWidgets.QPlainTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setStyleSheet("background: #1e1e1e; color: #00ff00; font-family: Consolas, monospace;")
        
        btn_clear = QtWidgets.QPushButton("Clear Log")
        btn_clear.clicked.connect(self.text_edit.clear)
        
        layout.addWidget(self.text_edit)
        layout.addWidget(btn_clear)

    def add_log(self, data):
        self.text_edit.moveCursor(QtGui.QTextCursor.End)
        self.text_edit.insertPlainText(str(data) + "\n")
        self.text_edit.moveCursor(QtGui.QTextCursor.End)

# --- 2. Widget UI ---
class MonitorWidgetUI(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background: transparent;")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.lbl_value = QtWidgets.QLabel("No Data")
        self.lbl_value.setStyleSheet("color: #00ff00; font-weight: bold; background: rgba(0,0,0,150); padding: 4px; border-radius: 4px; border: 1px solid #444;")
        self.lbl_value.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_value.setWordWrap(True)
        
        self.btn_open = QtWidgets.QPushButton("Open Console")
        self.btn_open.setStyleSheet("""
            QPushButton { background: #333; color: #ccc; border: 1px solid #555; padding: 2px; border-radius: 2px; font-size: 10px; }
            QPushButton:hover { background: #555; color: white; }
        """)
        
        layout.addWidget(self.lbl_value)
        layout.addWidget(self.btn_open)

# --- 3. Node Widget Wrapper ---
class MonitorNodeWidget(NodeBaseWidget):
    def __init__(self, parent=None):
        super(MonitorNodeWidget, self).__init__(parent)
        self.set_name('MonitorView') 
        self.set_label('')           
        self._ui = MonitorWidgetUI()
        self.set_custom_widget(self._ui)

    # These methods are required for graph saving/loading
    def get_value(self):
        return self._ui.lbl_value.text()

    def set_value(self, value):
        self._ui.lbl_value.setText(str(value))

# --- 4. The Node ---
class MonitorNode(BaseNode):
    __identifier__ = 'nodes.utility'
    NODE_NAME = 'MonitorNode'

    def __init__(self):
        super(MonitorNode, self).__init__()
        self.add_input('in', color=(255, 255, 0))
        
        self.log_window = NodeLogWindow(self.name())
        self.monitor_widget_wrapper = MonitorNodeWidget(self.view)
        
        self.add_custom_widget(self.monitor_widget_wrapper, tab='widgets')

        ui = self.monitor_widget_wrapper.get_custom_widget()
        ui.btn_open.clicked.connect(self.show_log_window)

    def show_log_window(self):
        if self.log_window.isVisible():
            self.log_window.hide()
        else:
            self.log_window.show()

    def update_data(self, data):
        clean_text = str(data).strip()
        if "]" in clean_text:
            clean_text = clean_text.split("]")[-1].strip()
            
        display_text = clean_text[:25] + "..." if len(clean_text) > 25 else clean_text
        self.monitor_widget_wrapper.set_value(display_text)
        self.log_window.add_log(data)