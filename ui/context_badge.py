"""Бейдж текущего контекста исполнения в верхней панели."""
from PySide6 import QtWidgets, QtCore

from core.execution_context import ExecutionContext


class ContextBadge(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumWidth(220)
        self.set_context(ExecutionContext())

    def set_context(self, ctx):
        bg, fg = ctx.palette()
        border = "3px solid #ffeb3b" if ctx.real_robot else "1px solid #222"
        self.setText(ctx.label())
        self.setToolTip(ctx.describe())
        self.setStyleSheet(
            f"QLabel {{ background-color: {bg}; color: {fg}; border: {border};"
            f" border-radius: 3px; padding: 4px 12px; font-weight: bold; }}")
