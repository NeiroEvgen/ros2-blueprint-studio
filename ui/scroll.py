"""Wraps a panel in a scrollable area."""
from PySide6 import QtWidgets


def scrollable(widget):
    """
    Puts the panel into a QScrollArea. Why: the minimum height of the bottom dock equalled
    the height of the "tallest" tab (Remote ~330 px), and the window could not be made lower
    than ~570 px. Inside a scroll area the panel can shrink, and a scroll bar appears
    when space runs out.
    """
    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.NoFrame)
    area.setWidget(widget)
    area.inner = widget            # access to the panel itself
    return area
