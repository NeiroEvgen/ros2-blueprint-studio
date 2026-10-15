"""
apt_safe — единственный способ звать apt в ЖИВОМ контейнере.

Зачем: фоновая установка сессии, rosdep, foxglove и кнопка Install во
вкладке Library запускаются параллельно через exec_run и дерутся за
dpkg/apt lock. Раньше retry-логика была скопирована в каждое место
по-своему (где-то 30 попыток, где-то fuser, где-то ничего).

Правила модуля:
- Ретраим ТОЛЬКО ошибки блокировки. Опечатка в имени пакета падает
  сразу, а не крутится минуту.
- Имена пакетов валидируются: в команду не попадёт ничего, кроме
  допустимого имени Debian-пакета (защита от кавычек/инъекций из поля
  ввода Library).
- Итоговый exit code значимый: 0 только при реальном успехе.

Dockerfile-строки `RUN apt-get ...` (dockerfile_manager, export_manager)
сюда НЕ относятся: они выполняются при сборке образа последовательно,
гонки за lock там нет.
"""
import re

DEFAULT_RETRIES = 30
DEFAULT_DELAY = 2

# Имя Debian-пакета: строчные буквы/цифры, затем [a-z0-9+.-].
# Опционально версия через '=' (напр. foo=1.2.3-1).
_PKG_RE = re.compile(r"^[a-z0-9][a-z0-9+.\-]+(=[A-Za-z0-9.+:~\-]+)?$")

# По этим фразам apt сообщает, что занят другим процессом.
_LOCK_PATTERN = "could not get lock|unable to acquire|is another process using it|unable to lock"


class InvalidPackageName(ValueError):
    pass


def validate_packages(packages):
    """Возвращает отсортированный список валидных имён или бросает InvalidPackageName."""
    pkgs = sorted({p.strip() for p in (packages or []) if p and p.strip()})
    bad = [p for p in pkgs if not _PKG_RE.match(p)]
    if bad:
        raise InvalidPackageName(f"недопустимые имена пакетов: {bad}")
    return pkgs


def _retry_block(cmd, label, retries, delay):
    # Одинарных кавычек внутри нет намеренно: сниппет можно вкладывать
    # в `bash -c '...'`. $rc/$out экранированы от внешней оболочки не нужно —
    # мы отдаём argv-список в exec_run, а не строку для shell.
    return (
        f'rc=1; for i in $(seq 1 {retries}); do '
        f'out=$({cmd} 2>&1); rc=$?; echo "$out"; '
        f'if [ $rc -eq 0 ]; then break; fi; '
        f'if ! echo "$out" | grep -qiE "{_LOCK_PATTERN}"; then break; fi; '
        f'echo "[apt_safe] {label}: lock busy, retry $i/{retries}"; sleep {delay}; '
        f'done; '
    )


def apt_snippet(packages=None, update=True, retries=DEFAULT_RETRIES, delay=DEFAULT_DELAY):
    """
    Bash-фрагмент: (опционально) apt-get update + apt-get install с ретраями
    только на блокировке. Завершается кодом последней операции.

    Предназначен для встраивания в более длинный скрипт (как в
    run_project_launch, где за apt идёт rosdep). Для отдельного вызова
    используй apt_cmd().
    """
    pkgs = validate_packages(packages)
    parts = ["export DEBIAN_FRONTEND=noninteractive; "]
    if update:
        parts.append(_retry_block("apt-get update", "update", retries, delay))
        if pkgs:
            parts.append("if [ $rc -eq 0 ]; then ")
    if pkgs:
        parts.append(_retry_block(f"apt-get install -y {' '.join(pkgs)}",
                                  "install", retries, delay))
        if update:
            parts.append("fi; ")
    parts.append("test $rc -eq 0")
    return "".join(parts)


def apt_cmd(packages=None, update=True, retries=DEFAULT_RETRIES, delay=DEFAULT_DELAY):
    """argv-список для docker exec_run: без shell-экранирования вообще."""
    return ["bash", "-c", apt_snippet(packages, update, retries, delay)]


def missing_check_snippet(packages):
    """
    Bash-фрагмент: печатает имена НЕустановленных пакетов, по одному в строке.
    Пустой вывод = всё уже стоит, apt звать не нужно.
    """
    pkgs = validate_packages(packages)
    if not pkgs:
        return "true"
    names = " ".join(pkgs)
    return (f'for p in {names}; do '
            f'dpkg -s "$p" >/dev/null 2>&1 || echo "$p"; '
            f'done')
