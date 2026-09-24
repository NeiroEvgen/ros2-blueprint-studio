"""
Где сейчас исполняются команды студии.

Самая частая ошибка при работе с роботом — потерять понимание, где ты:
на своей машине, на удалённом хосте или внутри контейнера. Команды летят
не туда: `docker` внутри контейнера, Windows-путь в scp из SSH-сессии,
`ros2 launch` не в том окружении. Единственным ориентиром было
приглашение шелла, и его легко проглядеть.

Модель чистая (без Qt), чтобы её можно было тестировать и переиспользовать
в будущем remote workspace (раздел 2.2+).
"""
from dataclasses import dataclass
from enum import Enum


class ContextKind(Enum):
    LOCAL = "local"          # сессии нет, всё на машине со студией
    SSH = "ssh"              # подключены к удалённому Docker-хосту, сессии нет
    CONTAINER = "container"  # есть живая сессия в контейнере


@dataclass(frozen=True)
class ExecutionContext:
    kind: ContextKind = ContextKind.LOCAL
    host: str = ""            # "" = эта машина; иначе user@host
    container: str = ""       # имя контейнера сессии
    real_robot: bool = False  # цель — реальное железо, а не симуляция

    # ---- представление ------------------------------------------------

    @property
    def where(self):
        return self.host or "local"

    def label(self):
        """Полная метка для бейджа и заголовка окна."""
        if self.kind is ContextKind.CONTAINER:
            base = f"CONTAINER: {self.container} @ {self.where}"
        elif self.kind is ContextKind.SSH:
            base = f"SSH: {self.host}"
        else:
            base = "LOCAL"
        return f"⚠ REAL ROBOT · {base}" if self.real_robot else base

    def short(self):
        """Короткая метка для каждой строки System Log."""
        if self.kind is ContextKind.CONTAINER:
            base = f"{self.container}@{_short_host(self.where)}"
        elif self.kind is ContextKind.SSH:
            base = f"ssh:{_short_host(self.host)}"
        else:
            base = "local"
        return f"[⚠ROBOT {base}]" if self.real_robot else f"[{base}]"

    def describe(self):
        """Развёрнутое объяснение для тултипа."""
        if self.kind is ContextKind.CONTAINER:
            where = "на этой машине" if not self.host else f"на удалённом хосте {self.host}"
            text = (f"Команды выполняются внутри контейнера «{self.container}» {where}.\n"
                    "docker / scp / Windows-пути отсюда недоступны.")
        elif self.kind is ContextKind.SSH:
            text = (f"Подключены к Docker на {self.host}, сессия не запущена.\n"
                    "Run поднимет контейнер на удалённой машине.")
        else:
            text = "Сессия не запущена. Команды выполняются на этой машине."
        if self.real_robot:
            text = ("⚠ ЦЕЛЬ — РЕАЛЬНЫЙ РОБОТ. Команды двигают железо.\n"
                    "Держи руку на аварийной кнопке.\n\n" + text)
        return text

    def palette(self):
        """(фон, текст) бейджа. Цвет дублирует текст, а не заменяет его."""
        if self.real_robot:
            return "#b71c1c", "#ffffff"
        return {
            ContextKind.LOCAL: ("#424242", "#e0e0e0"),
            ContextKind.SSH: ("#1565c0", "#ffffff"),
            ContextKind.CONTAINER: ("#2e7d32", "#ffffff"),
        }[self.kind]


def _short_host(host):
    """user@192.168.123.164 → .164 ; local → local ; имя хоста — как есть."""
    if not host or host == "local":
        return "local"
    h = host.split("@", 1)[-1]
    parts = h.split(".")
    if len(parts) == 4 and all(p.isdigit() for p in parts):
        return "." + parts[-1]
    return h


def host_from_target(target):
    """
    target из ContainerConfigStore: {"type": "local"|"ssh", "host": "ssh://user@h"}.
    Возвращает "user@h" для ssh и "" для локального.
    """
    if not target or target.get("type") != "ssh":
        return ""
    h = (target.get("host") or "").strip()
    return h[len("ssh://"):] if h.startswith("ssh://") else h


def resolve_context(target=None, session_active=False, container_name="",
                    real_robot=False):
    """Единая точка вычисления контекста из состояния студии."""
    host = host_from_target(target)
    if session_active:
        return ExecutionContext(ContextKind.CONTAINER, host, container_name, real_robot)
    if host:
        return ExecutionContext(ContextKind.SSH, host, "", real_robot)
    return ExecutionContext(ContextKind.LOCAL, "", "", real_robot)
