"""Обёртка панели в прокручиваемую область."""
from PySide6 import QtWidgets


def scrollable(widget):
    """
    Кладёт панель в QScrollArea. Зачем: минимальная высота нижнего дока равнялась
    высоте самой «высокой» вкладки (Remote ~330 px), и окно нельзя было сделать ниже
    ~570 px. В прокручиваемой области панель может сжиматься, а при нехватке места
    появляется полоса прокрутки.
    """
    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.NoFrame)
    area.setWidget(widget)
    area.inner = widget            # доступ к самой панели
    return area
