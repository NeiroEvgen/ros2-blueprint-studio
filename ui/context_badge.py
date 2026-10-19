"""Badge of the current execution context in the top bar."""
from PySide6 import QtWidgets, QtCore

from core.execution_context import ExecutionContext


class ContextBadge(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(QtCore.Qt.AlignCenter)
        # Explicit minimum: the label ("CONTAINER: name @ user@host" can be very long) must not
        # widen the top bar and prevent shrinking the window; when space runs out the badge shrinks
        # (full text in the tooltip). Do not use the Ignored policy: with it the layout
        # overlaps neighbouring buttons with the badge.
        self.setMinimumWidth(110)
        self.set_context(ExecutionContext())

    def set_context(self, ctx):
        bg, fg = ctx.palette()
        border = "3px solid #ffeb3b" if ctx.real_robot else "1px solid #222"
        self.setText(ctx.label())
        self.setToolTip(ctx.describe())
        self.setStyleSheet(
            f"QLabel {{ background-color: {bg}; color: {fg}; border: {border};"
            f" border-radius: 3px; padding: 4px 12px; font-weight: bold; }}")
