"""
UI fonts. The monospace font must not be "Consolas" alone: it does not exist on
Linux and macOS, and Qt substitutes a random proportional one. Hence a fallback stack.
"""
from PySide6 import QtGui

MONO_FAMILIES = ["Consolas", "Cascadia Mono", "DejaVu Sans Mono", "Menlo",
                 "Liberation Mono", "Courier New", "monospace"]

# for stylesheets: font-family: <MONO_CSS>;
MONO_CSS = ", ".join(f'"{f}"' if " " in f else f for f in MONO_FAMILIES)


def mono_font(point_size=10, bold=False):
    f = QtGui.QFont()
    f.setFamilies(MONO_FAMILIES)
    f.setStyleHint(QtGui.QFont.Monospace)
    f.setFixedPitch(True)
    f.setPointSize(point_size)
    f.setBold(bold)
    return f
