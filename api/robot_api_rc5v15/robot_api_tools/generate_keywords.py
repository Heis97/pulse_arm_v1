from __future__ import annotations

import argparse
import builtins
import importlib
import inspect
import json
import keyword
import sys
from pathlib import Path
from typing import Any, get_type_hints

PACKAGE_NAME: str = "API"
ROOT_SUBMODULES: list[str] = ["tools", "types", "coords", "io"]
OUTPUT_FILE_PATH: Path = Path("data/keywords.json")


def get_class_name_from_annotation(annotation: Any) -> str | None:
    if isinstance(annotation, str):
        return annotation
    elif hasattr(annotation, "__name__"):
        return annotation.__name__
    elif hasattr(annotation, "_name"):
        return annotation._name
    else:
        return str(annotation).split(".")[-1]


def extract_methods_and_params(cls: type[object]) -> list[str]:
    """Возвращает список методов и параметров класса."""
    words = set()
    for name in dir(cls):
        if name.startswith("_"):
            continue
        obj = getattr(cls, name)
        if inspect.isfunction(obj) or inspect.ismethod(obj):
            words.add(name)
            try:
                sig = inspect.signature(obj)
                params = [
                    param
                    for param in sig.parameters.keys()
                    if param not in ("self", "cls")
                ]
                words.update(params)
            except Exception:
                pass
    return sorted(words)


def build_class_tree(  #  noqa: C901
    cls: type[object], visited: set[str] | None = None, max_depth: int = 5
) -> dict[str, Any]:
    """
    Рекурсивно строит дерево класса с методами и подклассами.
    """
    if visited is None:
        visited = set()

    class_full_name = f"{cls.__module__}.{cls.__qualname__}"
    if class_full_name in visited or max_depth <= 0:
        return {}

    visited.add(class_full_name)

    tree = {
        "class": cls.__name__,
        "methods": extract_methods_and_params(cls),
    }

    # Получаем аннотации
    try:
        hints = get_type_hints(cls, localns={}, globalns={})
    except Exception:
        hints = getattr(cls, "__annotations__", {})

    for attr_name, annotation in hints.items():
        if attr_name.startswith("_"):
            continue

        class_name = get_class_name_from_annotation(annotation)
        if not class_name:
            continue

        found_cls = None
        try:
            if cls.__module__:
                mod = importlib.import_module(cls.__module__)
                if hasattr(mod, class_name):
                    found_cls = getattr(mod, class_name)
        except Exception:
            pass

        if found_cls is None:
            try:
                api_mod = importlib.import_module("API")
                if hasattr(api_mod, class_name):
                    found_cls = getattr(api_mod, class_name)
            except Exception:
                pass

        if found_cls is not None:
            subtree = build_class_tree(found_cls, visited, max_depth - 1)
            if subtree:
                tree[attr_name] = subtree
    return tree


def get_keywords_from_tree(tree: dict[str, Any], keywords: set[str]):
    for key, value in tree.items():
        if key in ("class", "methods"):
            if key == "methods":
                keywords.update(value)
            continue
        keywords.add(key)
        if isinstance(value, dict):
            get_keywords_from_tree(value, keywords)

    return keywords


def extract_from_module(module_name: str) -> set[str]:  # noqa: C901
    """Извлекает ключевые слова из модуля по его имени."""
    keywords = set()
    try:
        module = importlib.import_module(module_name)
    except Exception:
        return keywords

    # Получаем публичные имена
    public_names = getattr(
        module,
        "__all__",
        [name for name in dir(module) if not name.startswith("_")],
    )

    for name in public_names:
        if name.startswith("_"):
            continue
        try:
            obj = getattr(module, name)
        except AttributeError:
            continue

        keywords.add(name)

        # Если это класс — добавляем методы и параметры
        if inspect.isclass(obj):
            for method_name in dir(obj):
                if method_name.startswith("_"):
                    continue
                method = getattr(obj, method_name)
                if inspect.isfunction(method) or inspect.ismethod(method):
                    keywords.add(method_name)
                    try:
                        sig = inspect.signature(method)
                        keywords.update(sig.parameters.keys())
                    except Exception:
                        pass

        # Если это функция — добавляем её параметры
        elif inspect.isfunction(obj):
            try:
                sig = inspect.signature(obj)
                keywords.update(sig.parameters.keys())
            except Exception:
                pass

    return keywords


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="api-generate-keywords",
        description="Generate json file with package keywords",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=OUTPUT_FILE_PATH,
        dest="out_file",
        help=f"Path to the output file (default: {OUTPUT_FILE_PATH})",
    )

    args = parser.parse_args()

    all_keywords: set[str] = set()

    print("Adding Python reserved keywords...")
    all_keywords.update(keyword.kwlist)

    print("Adding Python built-in names...")
    all_keywords.update(dir(builtins))

    print(f"Adding root package name: '{PACKAGE_NAME}'")
    all_keywords.add(PACKAGE_NAME)

    # Extract from RobotApi
    print("Extracting keywords from RobotApi class...")
    api_module = importlib.import_module(PACKAGE_NAME)
    RobotApi = api_module.RobotApi
    api_tree = {"RobotApi": build_class_tree(RobotApi, max_depth=6)}
    get_keywords_from_tree(api_tree, all_keywords)

    # Extract from submodules
    for submodule in ROOT_SUBMODULES:
        full_name = f"{PACKAGE_NAME}.{submodule}"
        print(f"Extracting keywords from submodule: {full_name}")
        words = extract_from_module(full_name)
        all_keywords.update(words)

    print(f"Collected {len(all_keywords)} unique keywords in total.")

    # Ensure the output directory exists before writing
    args.out_file.parent.mkdir(parents=True, exist_ok=True)

    print(f"Saving keywords to '{args.out_file}'...")
    with open(args.out_file, "w", encoding="utf-8") as f:
        json.dump(sorted(all_keywords), f, indent=2, ensure_ascii=False)

    print("Done!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
