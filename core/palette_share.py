import os
import shutil
import zipfile
import yaml

# relative to the application folder, not the current one: started from a shortcut or another
# folder, a relative path pointed elsewhere and the palettes "disappeared"
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PALETTES_ROOT = os.path.join(APP_DIR, "nodes", "user_palettes")
STUDIO_VERSION = "0.6.0"


def export_palette(palette_name, zip_path):
    """Packs a palette into .bppalette. Returns the archive path."""
    src = os.path.join(PALETTES_ROOT, palette_name)
    if not os.path.isdir(src):
        raise FileNotFoundError(f"Palette '{palette_name}' not found")

    # stamp the studio version into every node.yaml (for future migrations)
    for node_dir in os.listdir(src):
        meta_path = os.path.join(src, node_dir, "node.yaml")
        if os.path.exists(meta_path):
            with open(meta_path, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f) or {}
            meta.setdefault('studio_version', STUDIO_VERSION)
            with open(meta_path, 'w', encoding='utf-8') as f:
                yaml.dump(meta, f, sort_keys=False, allow_unicode=True)

    if not zip_path.endswith(".bppalette"):
        zip_path += ".bppalette"
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in os.walk(src):
            for fname in files:
                full = os.path.join(root, fname)
                # inside the archive: PaletteName/NodeName/file
                arcname = os.path.relpath(full, PALETTES_ROOT)
                zf.write(full, arcname)
    return zip_path


def import_palette(zip_path, overwrite=False):
    """
    Unpacks a .bppalette into user_palettes.
    Returns (palette_name, node_list). Raises FileExistsError on a name clash.
    """
    with zipfile.ZipFile(zip_path, 'r') as zf:
        names = zf.namelist()

        # validation: at least one node.yaml, no path traversal
        if not any(n.endswith("node.yaml") for n in names):
            raise ValueError("Not a valid palette archive (no node.yaml inside)")
        for n in names:
            if n.startswith("/") or ".." in n:
                raise ValueError(f"Unsafe path in archive: {n}")

        # palette name = top folder of the archive
        top_dirs = {n.split("/")[0] for n in names if "/" in n}
        if len(top_dirs) != 1:
            raise ValueError("Archive must contain exactly one palette folder")
        palette_name = top_dirs.pop()

        target = os.path.join(PALETTES_ROOT, palette_name)
        if os.path.exists(target):
            if not overwrite:
                raise FileExistsError(palette_name)
            shutil.rmtree(target)

        os.makedirs(PALETTES_ROOT, exist_ok=True)
        zf.extractall(PALETTES_ROOT)

        nodes = sorted({n.split("/")[1] for n in names
                        if n.count("/") >= 2 and n.split("/")[1]})
        return palette_name, nodes