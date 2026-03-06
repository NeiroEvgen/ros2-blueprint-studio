import re
import datetime
from collections import deque
from PySide6 import QtWidgets, QtCore, QtGui

class TerminalLogDialog(QtWidgets.QDialog):
    
    def __init__(self, node_name, history, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Terminal: {node_name}")
        self.resize(800, 500)
        self.setStyleSheet("background-color: #0c0c0c;")

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Консольное поле
        self.text_view = QtWidgets.QTextEdit()
        self.text_view.setReadOnly(True)
        self.text_view.setFont(QtGui.QFont("Consolas", 10))
        self.text_view.setStyleSheet("""
            QTextEdit {
                background-color: #0c0c0c;
                color: #cccccc;
                border: 1px solid #333;
                padding: 5px;
            }
        """)
        
        # Заливаем историю
        self.text_view.setText("\n".join(history))
        self.text_view.moveCursor(QtGui.QTextCursor.End)
        
        layout.addWidget(self.text_view)
        
        # Кнопка закрытия в стиле терминала
        btn = QtWidgets.QPushButton("[ CLOSE ]")
        btn.setCursor(QtCore.Qt.PointingHandCursor)
        btn.setStyleSheet("""
            QPushButton {
                background-color: #0c0c0c; color: #444; border: none; font-family: Consolas; font-weight: bold;
            }
            QPushButton:hover { color: #fff; background-color: #222; }
        """)
        btn.clicked.connect(self.accept)
        layout.addWidget(btn)

class ConsoleDashboard(QtWidgets.QWidget):
    """
    Комбинированный виджет:
    Умеет переключаться между Умной Таблицей и Сырой Консолью по ПКМ.
    """
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Стек для переключения видов
        self.stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.stack)
        
        # ==========================================
        # 1. УМНАЯ ТАБЛИЦА (Smart Dashboard)
        # ==========================================
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["NODE / SOURCE", "LAST STATUS MESSAGE", "TIME"])
        self.table.setShowGrid(False)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setFocusPolicy(QtCore.Qt.NoFocus)
        
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        self.table.setColumnWidth(2, 90)
        
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #0c0c0c; color: #cccccc;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 10pt; border: none; gridline-color: #0c0c0c;
            }
            QHeaderView::section {
                background-color: #0c0c0c; color: #666666;
                border: none; border-bottom: 1px dashed #333;
                padding: 4px; font-weight: bold;
            }
            QTableWidget::item { border-bottom: 1px solid #1a1a1a; padding-left: 5px; }
            QTableWidget::item:selected { background-color: #264f78; color: #ffffff; }
        """)
        
        # ==========================================
        # 2. СЫРАЯ КОНСОЛЬ (Raw Terminal)
        # ==========================================
        self.raw_console = QtWidgets.QTextEdit()
        self.raw_console.setReadOnly(True)
        self.raw_console.setStyleSheet("""
            QTextEdit {
                background-color: #0c0c0c; color: #cccccc;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 10pt; border: none; padding: 5px;
            }
        """)
        
        # Добавляем оба виджета в стек
        self.stack.addWidget(self.table)
        self.stack.addWidget(self.raw_console)
        
        # Состояния
        self.is_raw_mode = False
        self.node_map = {}
        self.history = {}
        self.buffer = ""
        
        # ==========================================
        # СОБЫТИЯ И МЕНЮ
        # ==========================================
        self.table.cellDoubleClicked.connect(self.open_details)
        
        # Привязываем контекстное меню к обоим виджетам
        self.table.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.show_context_menu)
        
        self.raw_console.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.raw_console.customContextMenuRequested.connect(self.show_context_menu)

    def process_log(self, text):
        """Параллельная запись: и в сырую консоль, и в парсер таблицы"""
        # Чистим от цветовых символов ANSI
        clean_chunk = self._strip_ansi(text)
        
        # --- 1. ПИШЕМ В СЫРУЮ КОНСОЛЬ ---
        self.raw_console.moveCursor(QtGui.QTextCursor.End)
        self.raw_console.insertPlainText(clean_chunk)
        self.raw_console.verticalScrollBar().setValue(self.raw_console.verticalScrollBar().maximum())
        
        # --- 2. ПИШЕМ В УМНУЮ ТАБЛИЦУ ---
        full_text = self.buffer + clean_chunk
        
        if not full_text.endswith('\n'):
            lines = full_text.split('\n')
            self.buffer = lines[-1]
            lines = lines[:-1]
        else:
            lines = full_text.split('\n')
            self.buffer = ""

        for line in lines:
            clean_line = line.strip()
            if not clean_line: continue

            node_name = "System"
            msg = clean_line

            if "[launch]" in clean_line:
                node_name = "Launch"
                msg = re.sub(r'^\[.*?\]\s*\[launch\]:\s*', '', clean_line)
            else:
                match = re.match(r'^\[([a-zA-Z0-9_]+-\d+)\]\s*(.*)', clean_line)
                if match:
                    node_name = match.group(1)
                    msg = match.group(2)

            if node_name not in self.history:
                self.history[node_name] = deque(maxlen=2000)
            self.history[node_name].append(clean_line)

            display_msg = msg
            ros_log_match = re.search(r'^\[(INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\]\s*\[\d+\.\d+\]\s*\[.*?\]:\s*(.*)', msg)
            if ros_log_match:
                level = ros_log_match.group(1)
                actual_text = ros_log_match.group(2)
                display_msg = f"[{level}] {actual_text}"

            self._update_row(node_name, display_msg)

    def _update_row(self, name, msg):
        now = datetime.datetime.now().strftime("%H:%M:%S")
        status_color = "#cccccc"
        lower_msg = msg.lower()
        
        if "error" in lower_msg or "died" in lower_msg or "fail" in lower_msg: 
            status_color = "#ff3333"
        elif "warn" in lower_msg: 
            status_color = "#dddd00"
        elif "started" in lower_msg: 
            status_color = "#33ff33"

        if name not in self.node_map:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.node_map[name] = row
            
            item_name = QtWidgets.QTableWidgetItem(name)
            item_name.setForeground(QtGui.QColor("#4ec9b0"))
            item_name.setFont(QtGui.QFont("Consolas", 10, QtGui.QFont.Bold))
            self.table.setItem(row, 0, item_name)
            
            item_msg = QtWidgets.QTableWidgetItem(msg)
            item_msg.setForeground(QtGui.QColor(status_color))
            self.table.setItem(row, 1, item_msg)
            
            item_time = QtWidgets.QTableWidgetItem(now)
            item_time.setForeground(QtGui.QColor("#666666"))
            self.table.setItem(row, 2, item_time)
        else:
            row = self.node_map[name]
            
            item_msg = self.table.item(row, 1)
            item_msg.setText(msg)
            item_msg.setForeground(QtGui.QColor(status_color))
            
            self.table.item(row, 2).setText(now)

    def open_details(self, row, col):
        item = self.table.item(row, 0)
        if not item: return
        name = item.text()
        if name in self.history:
            dialog = TerminalLogDialog(name, self.history[name], self)
            dialog.exec()

    def show_context_menu(self, pos):
        """Всплывающее меню по правому клику"""
        menu = QtWidgets.QMenu(self)
        menu.setStyleSheet("""
            QMenu { background-color: #2b2b2b; color: white; border: 1px solid #444; }
            QMenu::item:selected { background-color: #007acc; }
        """)
        
        mode_text = "🔄 Switch to Raw Terminal" if not self.is_raw_mode else "🔄 Switch to Smart Dashboard"
        mode_action = menu.addAction(mode_text)
        menu.addSeparator()
        clear_action = menu.addAction("🧹 Clear All Logs")
        
        # Получаем глобальные координаты от виджета, по которому кликнули
        sender_widget = self.sender()
        global_pos = sender_widget.mapToGlobal(pos)
        
        action = menu.exec(global_pos)
        if action == mode_action:
            self.is_raw_mode = not self.is_raw_mode
            self.stack.setCurrentIndex(1 if self.is_raw_mode else 0)
        elif action == clear_action:
            self.clear()

    def _strip_ansi(self, text):
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)

    def clear(self):
        self.table.setRowCount(0)
        self.node_map.clear()
        self.history.clear()
        self.buffer = ""
        self.raw_console.clear()