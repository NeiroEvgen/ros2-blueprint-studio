"""
CLI: which files an entry point needs.

  python -m tools.import_closure path/to/node.py [--root DIR ...] [--json]
"""
import argparse
import json
import sys

from core import import_closure


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Python import closure of an entry file")
    ap.add_argument("entry")
    ap.add_argument("--root", action="append", help="search root (repeatable); default: entry dir")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    c = import_closure.compute(a.entry, a.root)
    data = {"local": c.local, "third_party": sorted(c.third_party), "stdlib": sorted(c.stdlib),
            "optional": sorted(c.optional), "dynamic": c.dynamic,
            "unresolved": c.unresolved, "errors": c.errors}
    if a.json:
        print(json.dumps(data, indent=2, ensure_ascii=False))
    else:
        for key, items in data.items():
            if items:
                print(f"[{key}]")
                for i in items:
                    print("  " + i)
    return 1 if (c.errors or c.unresolved) else 0


if __name__ == "__main__":
    sys.exit(main())
