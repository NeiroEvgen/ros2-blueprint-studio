"""
apt_safe: the only way to call apt in a LIVE container.

Why: the background session setup, rosdep, foxglove and the Install button in
the Library tab run in parallel via exec_run and fight over the
dpkg/apt lock. Waiting for the lock must be the same everywhere,
so it lives only here.

Module rules:
- Retry ONLY lock errors. A typo in a package name fails
  immediately instead of spinning for a minute.
- Package names are validated: nothing but a valid Debian package
  name can get into the command (protection against quotes/injection from the
  Library input field).
- The final exit code is meaningful: 0 only on real success.

Dockerfile lines `RUN apt-get ...` (dockerfile_manager, export_manager)
do NOT belong here: they run sequentially during the image build,
there is no race for the lock.
"""
import re

from core.i18n import t

DEFAULT_RETRIES = 30
DEFAULT_DELAY = 2

# Debian package name: lowercase letters/digits, then [a-z0-9+.-].
# Optionally a version after '=' (e.g. foo=1.2.3-1).
_PKG_RE = re.compile(r"^[a-z0-9][a-z0-9+.\-]+(=[A-Za-z0-9.+:~\-]+)?$")

# Phrases apt uses to say it is busy with another process.
_LOCK_PATTERN = "could not get lock|unable to acquire|is another process using it|unable to lock"


class InvalidPackageName(ValueError):
    pass


def validate_packages(packages):
    """Returns a sorted list of valid names or raises InvalidPackageName."""
    pkgs = sorted({p.strip() for p in (packages or []) if p and p.strip()})
    bad = [p for p in pkgs if not _PKG_RE.match(p)]
    if bad:
        raise InvalidPackageName(t("apt.invalid_names", names=bad))
    return pkgs


def _retry_block(cmd, label, retries, delay):
    # No single quotes inside on purpose: the snippet can be nested
    # in `bash -c '...'`. $rc/$out need no escaping from an outer shell:
    # we pass an argv list to exec_run, not a shell string.
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
    Bash snippet: (optionally) apt-get update + apt-get install with retries
    only on the lock. Exits with the code of the last operation.

    Meant to be embedded in a longer script (like
    run_project_launch, where rosdep follows apt). For a standalone call
    use apt_cmd().
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
    """argv list for docker exec_run: no shell escaping at all."""
    return ["bash", "-c", apt_snippet(packages, update, retries, delay)]


def missing_check_snippet(packages):
    """
    Bash snippet: prints the names of packages that are NOT installed, one per line.
    Empty output = everything is installed, no need to call apt.
    """
    pkgs = validate_packages(packages)
    if not pkgs:
        return "true"
    names = " ".join(pkgs)
    return (f'for p in {names}; do '
            f'dpkg -s "$p" >/dev/null 2>&1 || echo "$p"; '
            f'done')
