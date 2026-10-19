"""
Edit buffer for a file from any FileProvider.

Save never silently overwrites someone else's edit: on open we remember the sha1, on
save we compare it with the current content. Changed on the other side (someone
edited it on the robot) -> FileChangedExternally, and the UI asks whether to overwrite.
Same principle as the sync.
"""
import hashlib

from core.i18n import t

MAX_EDIT_BYTES = 2 * 1024 * 1024


class FileChangedExternally(Exception):
    pass


class NotEditable(Exception):
    pass


def _sha(data):
    return hashlib.sha1(data).hexdigest()


class EditBuffer:
    def __init__(self, provider, path):
        self.provider = provider
        self.path = path
        self.base_sha = None
        self.text = ""

    def open(self):
        data = self.provider.read_bytes(self.path)
        if len(data) > MAX_EDIT_BYTES:
            raise NotEditable(t("fb.too_big", mb=MAX_EDIT_BYTES // 1024 // 1024))
        if b"\0" in data:
            raise NotEditable(t("fb.binary"))
        try:
            self.text = data.decode("utf-8")
        except UnicodeDecodeError:
            raise NotEditable(t("fb.not_utf8"))
        self.base_sha = _sha(data)
        return self.text

    def is_dirty(self, current_text):
        return current_text.encode("utf-8") != self.text.encode("utf-8")

    def save(self, new_text, force=False):
        if not force and self.base_sha is not None and self.provider.exists(self.path):
            now = _sha(self.provider.read_bytes(self.path))
            if now != self.base_sha:
                raise FileChangedExternally(
                    t("fb.changed_remotely", path=self.path))
        data = new_text.encode("utf-8")
        self.provider.write_bytes(self.path, data)
        self.base_sha = _sha(data)
        self.text = new_text
