from PySide6 import QtWidgets, QtGui, QtCore

class CodeHighlighter(QtGui.QSyntaxHighlighter):
    """
    Syntax highlighter for Python and C++ code in the QPlainTextEdit.
    """
    def __init__(self, parent=None, mode='python'):
        super().__init__(parent)
        self.rules = []

        # --- Text Formats ---
        keyword_format = QtGui.QTextCharFormat()
        keyword_format.setForeground(QtGui.QColor("#569CD6"))  # VSCode Blue
        keyword_format.setFontWeight(QtGui.QFont.Bold)

        class_format = QtGui.QTextCharFormat()
        class_format.setForeground(QtGui.QColor("#4EC9B0"))  # Turquoise

        string_format = QtGui.QTextCharFormat()
        string_format.setForeground(QtGui.QColor("#CE9178"))  # Orange

        comment_format = QtGui.QTextCharFormat()
        comment_format.setForeground(QtGui.QColor("#6A9955"))  # Green
        
        # --- Keywords ---
        if mode == 'python':
            keywords = [
                "def", "class", "import", "from", "if", "else", "elif", 
                "return", "try", "except", "pass", "while", "for", "in", 
                "print", "self", "super", "None", "True", "False"
            ]
        else:  # cpp
            keywords = [
                "class", "public", "private", "protected", "void", "int", 
                "float", "double", "char", "string", "return", "if", "else", 
                "for", "while", "include", "using", "namespace", "auto", "const"
            ]

        # --- Rules ---
        for word in keywords:
            pattern = QtCore.QRegularExpression(r'\b' + word + r'\b')
            self.rules.append((pattern, keyword_format))

        # Strings ("..." and '...')
        self.rules.append((QtCore.QRegularExpression(r'".*"'), string_format))
        self.rules.append((QtCore.QRegularExpression(r"'.*'"), string_format))
        
        # Class Names (Capitalized words)
        self.rules.append((QtCore.QRegularExpression(r'\b[A-Z][a-zA-Z0-9_]+\b'), class_format))

        # Comments
        if mode == 'python':
            self.rules.append((QtCore.QRegularExpression(r'#.*'), comment_format))
        else:
            self.rules.append((QtCore.QRegularExpression(r'//.*'), comment_format))

    def highlightBlock(self, text):
        for pattern, format in self.rules:
            match_iter = pattern.globalMatch(text)
            while match_iter.hasNext():
                match = match_iter.next()
                self.setFormat(match.capturedStart(), match.capturedLength(), format)


class LineNumberArea(QtWidgets.QWidget):
    """
    Widget to display line numbers on the left side of the editor.
    """
    def __init__(self, editor):
        super().__init__(editor)
        self.codeEditor = editor

    def sizeHint(self):
        return QtCore.QSize(self.codeEditor.lineNumberAreaWidth(), 0)

    def paintEvent(self, event):
        self.codeEditor.lineNumberAreaPaintEvent(event)


class CodeEditor(QtWidgets.QPlainTextEdit):
    """
    Advanced text editor with line numbers and syntax highlighting.
    """
    def __init__(self, parent=None, mode='python'):
        super().__init__(parent)
        self.lineNumberArea = LineNumberArea(self)
        self.blockCountChanged.connect(self.updateLineNumberAreaWidth)
        self.updateRequest.connect(self.updateLineNumberArea)
        self.cursorPositionChanged.connect(self.highlightCurrentLine)
        self.updateLineNumberAreaWidth(0)
        
        # Font Configuration
        font = QtGui.QFont("Consolas", 11)
        font.setStyleHint(QtGui.QFont.Monospace)
        self.setFont(font)
        
        # Dark Theme Styles
        self.setStyleSheet("""
            QPlainTextEdit { background-color: #1e1e1e; color: #d4d4d4; border: none; }
        """)
        
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
        painter.fillRect(event.rect(), QtGui.QColor("#2d2d2d"))  # Gutter background

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


class AdvancedCodeDialog(QtWidgets.QDialog):
    """
    Dialog window wrapper for the code editor.
    """
    def __init__(self, code, language='python', parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Code Editor ({language})")
        self.resize(1000, 700)
        
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Toolbar
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