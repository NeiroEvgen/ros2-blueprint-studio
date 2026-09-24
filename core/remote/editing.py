"""
Буфер редактирования файла из любого FileProvider.

Save не затирает молча чужую правку: при открытии запоминаем sha1, при
сохранении сверяем с текущим содержимым. Изменилось на той стороне (кто-то
правил на роботе) → FileChangedExternally, UI спрашивает, перезаписывать ли.
Тот же принцип, что у синхронизации.
"""
import hashlib

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
            raise NotEditable(f"файл больше {MAX_EDIT_BYTES // 1024 // 1024} МБ")
        if b"\0" in data:
            raise NotEditable("двоичный файл")
        try:
            self.text = data.decode("utf-8")
        except UnicodeDecodeError:
            raise NotEditable("файл не в UTF-8")
        self.base_sha = _sha(data)
        return self.text

    def is_dirty(self, current_text):
        return current_text.encode("utf-8") != self.text.encode("utf-8")

    def save(self, new_text, force=False):
        if not force and self.base_sha is not None and self.provider.exists(self.path):
            now = _sha(self.provider.read_bytes(self.path))
            if now != self.base_sha:
                raise FileChangedExternally(
                    f"{self.path} изменён на той стороне после открытия")
        data = new_text.encode("utf-8")
        self.provider.write_bytes(self.path, data)
        self.base_sha = _sha(data)
        self.text = new_text
