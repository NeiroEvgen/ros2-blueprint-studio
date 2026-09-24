from PySide6 import QtWidgets, QtGui, QtCore

# === 1. ПОДСВЕТКА СИНТАКСИСА (Highlighter) ===
class CodeHighlighter(QtGui.QSyntaxHighlighter):
    def __init__(self, parent=None, mode='python'):
        super().__init__(parent)
        self.rules = []

        # Форматы
        keyword_format = QtGui.QTextCharFormat()
        keyword_format.setForeground(QtGui.QColor("#569CD6")) # Синий VSCode
        keyword_format.setFontWeight(QtGui.QFont.Bold)

        class_format = QtGui.QTextCharFormat()
        class_format.setForeground(QtGui.QColor("#4EC9B0")) # Бирюзовый

        string_format = QtGui.QTextCharFormat()
        string_format.setForeground(QtGui.QColor("#CE9178")) # Оранжевый

        comment_format = QtGui.QTextCharFormat()
        comment_format.setForeground(QtGui.QColor("#6A9955")) # Зеленый
        
        # yaml / xml / cmake — отдельные наборы правил (для файлового браузера).
        # Старые режимы python/cpp ниже не меняются.
        if mode == 'plain':
            return
        if mode in ('yaml', 'xml', 'cmake'):
            self._setup_extra(mode, keyword_format, string_format, comment_format, class_format)
            return

        # Ключевые слова
        if mode == 'python':
            keywords = [
                "def", "class", "import", "from", "if", "else", "elif", 
                "return", "try", "except", "pass", "while", "for", "in", 
                "print", "self", "super", "None", "True", "False"
            ]
        else: # cpp
            keywords = [
                "class", "public", "private", "protected", "void", "int", 
                "float", "double", "char", "string", "return", "if", "else", 
                "for", "while", "include", "using", "namespace", "auto", "const"
            ]

        # Правила
        for word in keywords:
            pattern = QtCore.QRegularExpression(r'\b' + word + r'\b')
            self.rules.append((pattern, keyword_format))

        # Строки ("...")
        self.rules.append((QtCore.QRegularExpression(r'".*"'), string_format))
        self.rules.append((QtCore.QRegularExpression(r"'.*'"), string_format))
        
        # Классы
        self.rules.append((QtCore.QRegularExpression(r'\b[A-Z][a-zA-Z0-9_]+\b'), class_format))

        # Комментарии
        if mode == 'python':
            self.rules.append((QtCore.QRegularExpression(r'#.*'), comment_format))
        else:
            self.rules.append((QtCore.QRegularExpression(r'//.*'), comment_format))

    def _setup_extra(self, mode, kw, string, comment, cls):
        R = QtCore.QRegularExpression
        if mode == 'yaml':
            self.rules.append((R(r'^\s*-?\s*[\w.\-/]+(?=\s*:)'), kw))
            self.rules.append((R(r'\b(true|false|null|True|False|yes|no)\b'), cls))
            self.rules.append((R(r'"[^"]*"|\'[^\']*\''), string))
            self.rules.append((R(r'#.*'), comment))
        elif mode == 'xml':
            self.rules.append((R(r'</?[\w:\-]+|/?>'), kw))
            self.rules.append((R(r'\b[\w:\-]+(?==)'), cls))
            self.rules.append((R(r'"[^"]*"'), string))
            self.rules.append((R(r'<!--.*-->'), comment))
        elif mode == 'cmake':
            self.rules.append((R(r'^\s*\w+(?=\s*\()'), kw))
            self.rules.append((R(r'\$\{[^}]*\}'), cls))
            self.rules.append((R(r'"[^"]*"'), string))
            self.rules.append((R(r'#.*'), comment))

    def highlightBlock(self, text):
        for pattern, format in self.rules:
            match_iter = pattern.globalMatch(text)
            while match_iter.hasNext():
                match = match_iter.next()
                self.setFormat(match.capturedStart(), match.capturedLength(), format)

# === 2. НОМЕРА СТРОК ===
class LineNumberArea(QtWidgets.QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.codeEditor = editor

    def sizeHint(self):
        return QtCore.QSize(self.codeEditor.lineNumberAreaWidth(), 0)

    def paintEvent(self, event):
        self.codeEditor.lineNumberAreaPaintEvent(event)

# === 3. САМ РЕДАКТОР ===
class CodeEditor(QtWidgets.QPlainTextEdit):
    def __init__(self, parent=None, mode='python'):
        super().__init__(parent)
        self.lineNumberArea = LineNumberArea(self)
        self.blockCountChanged.connect(self.updateLineNumberAreaWidth)
        self.updateRequest.connect(self.updateLineNumberArea)
        self.cursorPositionChanged.connect(self.highlightCurrentLine)
        self.updateLineNumberAreaWidth(0)
        
        # Шрифт
        font = QtGui.QFont("Consolas", 11)
        font.setStyleHint(QtGui.QFont.Monospace)
        self.setFont(font)
        
        # Цвета (Dark Theme)
        self.setStyleSheet("""
            QPlainTextEdit { background-color: #1e1e1e; color: #d4d4d4; border: none; }
        """)
        
        # Подключаем подсветку
        self.highlighter = CodeHighlighter(self.document(), mode)

    def lineNumberAreaWidth(self):
        digits = 1
        max_num = max(1, self.blockCount())
        while max_num >= 10:
            max_num /= 10
            digits += 1
        return 15 + self.fontMetrics().horizontalAdvance('9') * digits

    def updateLineNumberAreaWidth(self, _):
        self.setViewportMargins(self.lineNumberAreaWidth(), 0, 0, 0)

    def updateLineNumberArea(self, rect, dy):
        if dy:
            self.lineNumberArea.scroll(0, dy)
        else:
            self.lineNumberArea.update(0, rect.y(), self.lineNumberArea.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self.updateLineNumberAreaWidth(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self.lineNumberArea.setGeometry(QtCore.QRect(cr.left(), cr.top(), self.lineNumberAreaWidth(), cr.height()))

    def lineNumberAreaPaintEvent(self, event):
        painter = QtGui.QPainter(self.lineNumberArea)
        painter.fillRect(event.rect(), QtGui.QColor("#2d2d2d")) # Фон номеров

        block = self.firstVisibleBlock()
        blockNumber = block.blockNumber()
        top = self.blockBoundingGeometry(block).translated(self.contentOffset()).top()
        bottom = top + self.blockBoundingRect(block).height()

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                number = str(blockNumber + 1)
                painter.setPen(QtGui.QColor("#858585"))
                painter.drawText(0, int(top), self.lineNumberArea.width() - 5, self.fontMetrics().height(),
                                 QtCore.Qt.AlignRight, number)
            block = block.next()
            top = bottom
            bottom = top + self.blockBoundingRect(block).height()
            blockNumber += 1

    def highlightCurrentLine(self):
        extraSelections = []
        if not self.isReadOnly():
            selection = QtWidgets.QTextEdit.ExtraSelection()
            lineColor = QtGui.QColor("#2d2d2d")
            selection.format.setBackground(lineColor)
            selection.format.setProperty(QtGui.QTextFormat.FullWidthSelection, True)
            selection.cursor = self.textCursor()
            selection.cursor.clearSelection()
            extraSelections.append(selection)
        self.setExtraSelections(extraSelections)

# === 4. ДИАЛОГОВОЕ ОКНО ===
class AdvancedCodeDialog(QtWidgets.QDialog):
    def __init__(self, code, language='python', parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Code Editor ({language})")
        self.resize(1000, 700)
        
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        
        # Тулбар (можно добавить кнопки Save/Load)
        toolbar = QtWidgets.QFrame()
        toolbar.setStyleSheet("background: #333; height: 40px;")
        toolbar_layout = QtWidgets.QHBoxLayout(toolbar)
        
        lbl = QtWidgets.QLabel(f"Editing: {language.upper()}")
        lbl.setStyleSheet("color: white; font-weight: bold;")
        toolbar_layout.addWidget(lbl)
        toolbar_layout.addStretch()
        
        btn_save = QtWidgets.QPushButton("Save & Close")
        btn_save.setStyleSheet("background: #2e7d32; color: white; padding: 5px 15px; border: none;")
        btn_save.clicked.connect(self.accept)
        toolbar_layout.addWidget(btn_save)
        
        layout.addWidget(toolbar)
        
        self.editor = CodeEditor(mode=language)
        self.editor.setPlainText(code)
        layout.addWidget(self.editor)
        
    def get_code(self):
        return self.editor.toPlainText()

# === Режим подсветки по имени файла ===
def mode_for_path(path):
    name = path.replace("\\", "/").rsplit("/", 1)[-1].lower()
    if name == "cmakelists.txt" or name.endswith(".cmake"):
        return "cmake"
    ext = name.rsplit(".", 1)[-1] if "." in name else ""
    if ext in ("py",):
        return "python"
    if ext in ("cpp", "cc", "cxx", "hpp", "hh", "h", "c", "ino"):
        return "cpp"
    if ext in ("yaml", "yml"):
        return "yaml"
    if ext in ("xml", "urdf", "srdf", "xacro", "launch", "sdf"):
        return "xml"
    return "plain"
