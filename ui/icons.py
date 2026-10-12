"""
Векторные иконки интерфейса (SVG, рисуются в QPixmap нужного масштаба).

Зачем: эмодзи в подписях кнопок зависят от шрифтов ОС (на Linux/macOS/Windows они
выглядят по-разному или превращаются в «квадратики») и раздувают ширину кнопок.
SVG одинаков везде, чёткий на HiDPI и перекрашивается под состояние кнопки.

    btn.setIcon(icons.icon("save"))
"""
from PySide6 import QtCore, QtGui

try:                                        # QtSvg входит в обычную сборку PySide6
    from PySide6 import QtSvg
except ImportError:                          # pragma: no cover
    QtSvg = None

# Тела иконок: viewBox 24x24, обводка currentColor (подставляется при рисовании).
_BODIES = {
    "open":      '<path d="M3 7a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/>',
    "save":      '<path d="M5 3h11l3 3v13a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2z"/>'
                 '<path d="M7 3v6h8V3"/><path d="M7 21v-7h10v7"/>',
    "run":       '<path d="M7 4l13 8-13 8z" fill="currentColor"/>',
    "stop":      '<rect x="6" y="6" width="12" height="12" rx="1.5" fill="currentColor"/>',
    "visualize": '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
    "robot":     '<rect x="5" y="8" width="14" height="11" rx="2.5"/><path d="M12 4v4"/>'
                 '<circle cx="12" cy="3.2" r="1"/><circle cx="9.5" cy="13" r="1" fill="currentColor"/>'
                 '<circle cx="14.5" cy="13" r="1" fill="currentColor"/><path d="M3 12v3M21 12v3"/>',
    "upload":    '<path d="M12 16V4"/><path d="M7 9l5-5 5 5"/><path d="M4 20h16"/>',
    "download":  '<path d="M12 4v12"/><path d="M7 11l5 5 5-5"/><path d="M4 20h16"/>',
    "group":     '<rect x="3" y="3" width="18" height="18" rx="2" stroke-dasharray="3 3"/>'
                 '<rect x="8" y="8" width="8" height="8" rx="1"/>',
    "clear":     '<path d="M4 7h16"/><path d="M9 7V4h6v3"/><path d="M6 7l1 13h10l1-13"/>'
                 '<path d="M10 11v6M14 11v6"/>',
    "package":   '<path d="M21 8l-9-5-9 5v8l9 5 9-5z"/><path d="M3 8l9 5 9-5"/><path d="M12 13v8"/>',
    "package_in": '<path d="M21 8l-9-5-9 5v8l9 5 9-5z"/><path d="M12 8v6"/><path d="M9 11.5l3 3 3-3"/>',
    "back":      '<path d="M19 12H5"/><path d="M11 6l-6 6 6 6"/>',
    "plug":      '<path d="M9 3v5M15 3v5"/><path d="M6 8h12v4a6 6 0 0 1-12 0z"/><path d="M12 18v3"/>',
    "unplug":    '<path d="M9 3v5M15 3v5"/><path d="M6 8h12v4a6 6 0 0 1-12 0z"/><path d="M4 4l16 16"/>',
    "sync":      '<path d="M4 8h13l-3-3"/><path d="M20 16H7l3 3"/>',
    "build":     '<path d="M14 6a4 4 0 0 0 5 5l-8 8a2.1 2.1 0 0 1-3-3z"/><path d="M14 6l3-3 3 3"/>',
    "more":      '<circle cx="5" cy="12" r="1.7" fill="currentColor"/><circle cx="12" cy="12" r="1.7" fill="currentColor"/>'
                 '<circle cx="19" cy="12" r="1.7" fill="currentColor"/>',
    "terminal":  '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M7 10l3 2-3 2"/><path d="M12 15h5"/>',
}

_cache = {}


def names():
    return sorted(_BODIES)


def svg(name, color="#ffffff", stroke=2.0):
    """Готовый текст SVG (для тестов и для экспорта)."""
    body = _BODIES[name].replace("currentColor", color)
    return ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" '
            f'stroke="{color}" stroke-width="{stroke}" stroke-linecap="round" '
            f'stroke-linejoin="round">{body}</svg>')


def _pixmap(name, color, px):
    pm = QtGui.QPixmap(px, px)
    pm.fill(QtCore.Qt.transparent)
    if QtSvg is None:
        return pm
    renderer = QtSvg.QSvgRenderer(QtCore.QByteArray(svg(name, color).encode("utf-8")))
    painter = QtGui.QPainter(pm)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    renderer.render(painter)
    painter.end()
    return pm


def icon(name, color="#ffffff", disabled_color="#666666", size=18):
    """QIcon: нормальное и неактивное состояние, чёткий на HiDPI (1x/2x/3x)."""
    key = (name, color, disabled_color, size)
    if key in _cache:
        return _cache[key]
    ic = QtGui.QIcon()
    for scale in (1, 2, 3):
        px = size * scale
        ic.addPixmap(_pixmap(name, color, px), QtGui.QIcon.Normal)
        ic.addPixmap(_pixmap(name, disabled_color, px), QtGui.QIcon.Disabled)
    _cache[key] = ic
    return ic
