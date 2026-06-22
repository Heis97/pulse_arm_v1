"""
Cython-Ready Monolithic Bundler.

This script provides a tool to flatten a Python package into a single,
optimized, and Cython-compatible source file. It performs advanced AST
transformations to ensure the resulting monolith is clean, efficient,
and strictly ordered.

Key Features:
- Topological Sorting: Guarantees strict execution order (dependencies
  are always defined before dependents).
- Cycle Resolution: Automatically breaks circular dependencies (e.g.,
  in decorators) by injecting lazy import helpers.
- Import Optimization: Hoists, deduplicates, and alphabetically sorts
  all third-party imports at the top of the file.
- Namespace Preservation: Generates static proxy classes to maintain
  correct attribute access for aliased imports (e.g., `import pkg as alias`).
- Code Cleanup: Strips all `__all__` declarations and `if TYPE_CHECKING:`
  blocks to minimize the final file size.
- Future Annotations: Ensures exactly one `from __future__ import annotations`
  is injected at the very beginning of the file.
"""

from __future__ import annotations

import argparse
import ast
import importlib.metadata
import importlib.util
import re
import subprocess
import sys
import traceback
from collections import defaultdict, deque
from datetime import datetime, timezone
from pathlib import Path
from typing import TypedDict

SRC_DIR: str = "API"
PACKAGE_ROOT: str = "API"
ENTRY_POINT: str = "rc_api:RobotApi"
OUT_FILE: str = "robot_api_monolith.py"

if sys.version_info < (3, 10):
    sys.exit(
        f"ERROR: For script execution required python 3.10+.\n"
        f"   Current version: {sys.version.split()[0]}"
    )


class ParsedModuleInfo(TypedDict):
    body: list[ast.AST]
    external_imports: list[ast.Import | ast.ImportFrom]
    has_future: bool
    exports: set[str]
    deps: set[str]
    aliases: dict[str, str]
    path: Path


class DocstringRemover(ast.NodeTransformer):
    """
    An AST node transformer that removes docstrings from Python source code.

    In Python, docstrings are represented as the first `ast.Expr` node containing
    a string constant within the body of a module, class, or function. This
    transformer identifies and removes these specific nodes, effectively stripping
    all documentation strings to reduce bundle size or hide internal API details.
    """

    def _strip(self, node):
        """
        Removes the docstring from the given AST node if it exists.

        Checks if the first statement in the node's body is an expression
        consisting of a string constant. If the condition is met, the node
        is removed from the body.

        Args:
            node: The AST node (e.g., Module, ClassDef, FunctionDef) to process.
        """
        if (
            node.body
            and isinstance(node.body[0], ast.Expr)
            and isinstance(node.body[0].value, ast.Constant)
            and isinstance(node.body[0].value.value, str)
        ):
            node.body.pop(0)

    def visit_FunctionDef(self, node: ast.FunctionDef):
        self._strip(node)
        return self.generic_visit(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef):
        self._strip(node)
        return self.generic_visit(node)

    def visit_ClassDef(self, node: ast.ClassDef):
        self._strip(node)
        return self.generic_visit(node)

    def visit_Module(self, node: ast.Module):
        self._strip(node)
        return self.generic_visit(node)


def find_source_dir() -> Path | None:
    """
    Attempts to find the source directory:
    1. Checks if 'SRC_DIR' exists in the current working directory
    2. Falls back to the installed package location
    """
    cwd_api = Path.cwd() / SRC_DIR
    if cwd_api.is_dir():
        return cwd_api

    try:
        spec = importlib.util.find_spec(PACKAGE_ROOT)
        if spec and spec.origin:
            return Path(spec.origin).parent
    except (ImportError, ModuleNotFoundError, ValueError):
        pass

    return None


def build_mod_map(src_dir: Path, pkg_root: str) -> dict[str, Path]:
    """
    Builds a mapping of fully qualified module names to their file paths.

    Recursively scans the source directory for `.py` files, converts their
    relative paths into dotted module notation (e.g., 'core/network.py' -> 'core.network'),
    correctly handles `__init__.py` files, and prepends the root package name.

    Args:
        src_dir: Root directory containing the source code.
        pkg_root: Base package name (e.g., 'API' or 'my_package').
    """

    mod_map = {}
    for py in sorted(src_dir.rglob("*.py")):
        rel = py.relative_to(src_dir)
        parts = list(rel.parts)
        parts[-1] = parts[-1].replace(".py", "")
        mod_name = ".".join(parts)
        if mod_name.endswith(".__init__"):
            mod_name = mod_name.rsplit(".__init__", 1)[0]
        full = f"{pkg_root}.{mod_name}" if mod_name else pkg_root
        mod_map[full] = py
    return mod_map


def resolve_dep(
    current_mod: str, node: ast.ImportFrom, pkg_root: str, mod_map: dict
) -> str | None:
    """
    Resolves the fully qualified module name from an `ast.ImportFrom` node.

    Handles both relative (e.g., `from . import x`, `from ..core import y`)
    and absolute imports within the specified package root. It calculates the
    target module path based on the current module's location and the import
    level, then attempts to find an exact or partial match in the module map.

    Args:
        current_mod: The fully qualified name of the module containing the import.
        node: The AST node representing the `from ... import ...` statement.
        pkg_root: The root package name (e.g., 'API').
        mod_map: A mapping of fully qualified module names to their file paths.

    Returns:
        The resolved fully qualified module name (str) if it exists within the
        project's `mod_map`. Returns `None` if the import is out-of-bounds,
        invalid, or targets an external third-party library.
    """
    level = node.level
    module = node.module or ""
    parts = current_mod.split(".")
    path = mod_map.get(current_mod)
    is_pkg = path and path.name == "__init__.py"
    base = list(parts)
    pop_count = level - 1 if is_pkg else level
    for _ in range(pop_count):
        if base:
            base.pop()
        else:
            return None
    if module:
        base.append(module)
    target = ".".join(base) if base else pkg_root
    if not target.startswith(pkg_root):
        target = f"{pkg_root}.{target}"
    if target in mod_map:
        return target

    # Fallback: check if the target is a parent package of any known module
    for m in mod_map:
        if m == target or m.startswith(target + "."):
            return m
    return None


def is_type_checking_block(node: ast.AST) -> bool:
    """
    Determines whether a given AST node represents a `TYPE_CHECKING` conditional block.

    This is used to identify and safely remove type-checking blocks during bundling,
    as they are not needed at runtime and only increase the final file size.

    Matches both common patterns:
      - `if TYPE_CHECKING:` (ast.Name)
      - `if typing.TYPE_CHECKING:` (ast.Attribute)

    Args:
        node: The AST node to evaluate (typically expected to be an `ast.If` node).

    Returns:
        True if the node is a `TYPE_CHECKING` block, False otherwise.
    """
    if isinstance(node, ast.If):
        test = node.test
        if isinstance(test, ast.Name) and test.id == "TYPE_CHECKING":
            return True
        if isinstance(test, ast.Attribute) and test.attr == "TYPE_CHECKING":
            return True
    return False


def parse_module(  # noqa: C901
    mod_name: str, path: Path, pkg_root: str, mod_map: dict
) -> ParsedModuleInfo | None:
    """
    Parses a Python module's AST to extract metadata and filter runtime-irrelevant code.

    This is a core function for the bundling process. It reads the source file,
    parses it into an Abstract Syntax Tree (AST), and performs the following:
      1. Filters out `TYPE_CHECKING` blocks and `__future__` imports.
      2. Identifies public exports (classes, functions, variables, and imported names).
      3. Resolves and collects internal dependencies (relative and absolute) using `mod_map`.
      4. Separates third-party `external_imports` from the main executable `body`.
      5. Ignores `__all__` assignments in the main body.

    If the file contains a SyntaxError, it safely returns an empty dictionary
    to prevent the bundler from crashing on invalid files.

    Args:
        mod_name: The fully qualified name of the module being parsed.
        path: The file path to the source code.
        pkg_root: The root package name (e.g., 'API') used to distinguish
                  internal imports from external ones.
        mod_map: A mapping of fully qualified module names to their file paths,
                 used for resolving relative imports.
    """
    src = path.read_text("utf-8")
    try:
        tree = ast.parse(src, filename=str(path))
    except SyntaxError:
        return None

    filtered_body = []
    has_future = False
    exports = set()
    deps = set()
    aliases = {}
    external_imports = []

    for node in tree.body:
        if is_type_checking_block(node):
            continue
        if isinstance(node, ast.ImportFrom) and node.module == "__future__":
            has_future = True
            continue

        # Collect public names
        if isinstance(
            node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)
        ):
            exports.add(node.name)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and not t.id.startswith("_"):
                    exports.add(t.id)
        elif isinstance(node, ast.ImportFrom):
            for a in node.names:
                exports.add(a.asname or a.name)
        elif isinstance(node, ast.Import):
            for a in node.names:
                exports.add(a.asname or a.name.split(".")[-1])

        # Collect internal imports
        is_internal = False
        if isinstance(node, ast.ImportFrom):
            is_relative = node.level and node.level > 0
            is_absolute_internal = node.module and node.module.startswith(
                pkg_root
            )
            if is_relative or is_absolute_internal:
                is_internal = True
                if is_relative and node.module is None:
                    for a in node.names:
                        sub_target = resolve_dep(
                            mod_name,
                            ast.ImportFrom(
                                level=node.level, module=a.name, names=[]
                            ),
                            pkg_root,
                            mod_map,
                        )
                        if sub_target and sub_target in mod_map:
                            deps.add(sub_target)
                else:
                    target = (
                        resolve_dep(mod_name, node, pkg_root, mod_map)
                        if is_relative
                        else node.module
                    )
                    if target and target in mod_map:
                        deps.add(target)
        elif isinstance(node, ast.Import):
            for a in node.names:
                name = a.name
                if name.startswith(pkg_root):
                    for m in mod_map:
                        if name == m or name.startswith(m + "."):
                            deps.add(m)
                            if a.asname:
                                aliases[a.asname] = m
                            is_internal = True
                            break

        if not is_internal:
            # Skipping __all__
            skip_all = False
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == "__all__":
                        skip_all = True
                        break
            if skip_all:
                continue

            if isinstance(node, (ast.Import, ast.ImportFrom)):
                external_imports.append(node)
            else:
                filtered_body.append(node)

    return ParsedModuleInfo(
        body=filtered_body,
        external_imports=external_imports,
        has_future=has_future,
        exports=exports,
        deps=deps,
        aliases=aliases,
        path=path,
    )


def find_entry_module(
    entry: str, parsed_modules: dict[str, ParsedModuleInfo | None]
) -> str:
    """
    Resolves an entry point string to a fully qualified module name.

    Parses the `entry` argument (which can be a module path like 'API.rc_api'
    or include an object name like 'API.rc_api:RobotApi') and searches through
    the `parsed_modules` dictionary to find the correct target module.

    The resolution follows a two-step fallback strategy:
      1. Exact/Partial Match: Looks for a module ending with `mod_part`.
         If `name_part` is provided, verifies it exists in the module's exports.
      2. Global Fallback: If no specific module matched, searches all parsed
         modules for `name_part` in their exports.

    Args:
        entry: The entry point specification (e.g., 'module.path' or 'module.path:Object').
        parsed_modules: A dictionary mapping fully qualified module names to their
                        parsed metadata (as returned by `parse_module`).
                        Values may be `None` if parsing failed.
    """
    if ":" in entry:
        mod_part, name_part = entry.split(":", 1)
    else:
        mod_part, name_part = entry, None
    for mod_name in parsed_modules:
        p = parsed_modules[mod_name]
        if p is None:
            continue
        if mod_name.endswith(f".{mod_part}") or mod_name == mod_part:
            if not name_part or name_part in p["exports"]:
                print(f" -> Found '{name_part}' in '{mod_name}'")
                return mod_name

    if name_part:
        for mod_name, p in parsed_modules.items():
            if p and name_part in p["exports"]:
                print(f" -> Found '{name_part}' in '{mod_name}'")
                return mod_name
    available = [k for k, v in parsed_modules.items() if v]
    raise ValueError(
        f"Entry '{entry}' not found. Available modules: {available[:10]}..."
    )


def collect_reachable(
    start_mod: str, parsed_modules: dict[str, ParsedModuleInfo | None]
) -> tuple[set[str], dict[str, set[str]], dict[str, str]]:
    """
    Traverses the module dependency graph using Breadth-First Search (BFS).

    Starting from the `start_mod`, this function discovers all reachable internal
    modules, builds an adjacency list representing the dependency graph, and
    aggregates all import aliases encountered along the way.

    This is a critical step for determining the minimal set of files required
    for the final monolithic bundle and resolving naming conflicts via aliases.

    Args:
        start_mod: The fully qualified name of the entry module to start traversal from.
        parsed_modules: Dictionary mapping module names to their parsed metadata.
                        Values may be `None` if parsing previously failed.

    Returns:
        A tuple containing:
        - visited: A set of all reachable, fully qualified module names.
        - graph: An adjacency list (dependency graph) mapping each module to a
                 set of its direct internal dependencies.
        - all_aliases: A merged dictionary of all import aliases found across
                       the reachable modules (e.g., {'alias_name': 'resolved.module'}).
    """
    visited, queue, graph, all_aliases = (
        set(),
        deque([start_mod]),
        defaultdict(set),
        {},
    )
    while queue:
        mod = queue.popleft()
        if mod in visited:
            continue
        visited.add(mod)
        p = parsed_modules.get(mod)
        if not p:
            continue
        for dep in p["deps"]:
            if dep in parsed_modules:
                graph[mod].add(dep)
                queue.append(dep) if dep not in visited else None
        all_aliases.update(p["aliases"])
    return visited, graph, all_aliases


def topo_sort_with_cycle_break(  # noqa: C901
    graph: dict[str, set[str]], nodes: set[str]
) -> tuple[list[str], list[str]]:
    """
    Performs topological sorting of modules with heuristic cycle resolution.

    Uses a variation of Kahn's algorithm to order modules based on their
    dependencies. Unlike standard topological sort, this function gracefully
    handles circular dependencies (cycles) by attempting to find the safest
    insertion point for cyclic modules, prioritizing those with fewer dependencies
    (e.g., utilities or decorators).

    Args:
        graph: An adjacency list mapping each module to a set of its dependencies.
        nodes: The set of module names to be sorted.

    Returns:
        A tuple containing:
        - order: A list of module names in topologically sorted order.
        - remaining: A list of module names that were part of circular dependencies
                     and had to be placed heuristically.
    """
    in_degree = {n: 0 for n in nodes}
    reverse = defaultdict(set)
    for n, deps in graph.items():
        for d in deps:
            if d in nodes:
                in_degree[n] += 1
                reverse[d].add(n)

    queue = deque(sorted([n for n in nodes if in_degree[n] == 0]))
    order, remaining = [], []
    while queue:
        n = queue.popleft()
        order.append(n)
        for dependent in sorted(reverse.get(n, [])):
            in_degree[dependent] -= 1
            if in_degree[dependent] == 0:
                queue.append(dependent)

    remaining = sorted([n for n in nodes if n not in order])
    if remaining:
        print(f"! Circular dependencies detected: {len(remaining)} modules")
        print(
            " -> Automatically breaking cycles (heuristically placing utilities/decorators first)..."
        )
        remaining.sort(key=lambda m: len(graph.get(m, set())))
        for m in remaining:
            insert_idx = len(order)
            for i, placed in enumerate(order):
                deps_met = all(
                    d in order[:i] or d in remaining
                    for d in graph.get(m, set())
                )
                if deps_met:
                    insert_idx = i
                    break
            order.insert(insert_idx, m)
    return order, remaining


def break_cyclic_imports_in_code(  # noqa: C901
    code_str: str, cyclic_targets: set[str], pkg_root: str
) -> str:
    """
    Modifies Python source code to resolve circular dependencies via lazy importing.

    Parses the source code into an AST, identifies top-level imports targeting
    modules listed in `cyclic_targets`, and removes them from the main execution flow.
    Instead, it generates a helper function `_resolve_cycles()` that performs
    deferred (lazy) imports and assigns them to the global namespace.

    The helper function definition is inserted near the top of the module, and
    a call to it is appended at the end of the file.

    Args:
        code_str: The original Python source code as a string.
        cyclic_targets: A set of fully qualified module names involved in circular
                        dependencies.
        pkg_root: The root package name to ensure only internal imports are modified.

    Returns:
        The modified Python source code string with cyclic imports resolved.
    """
    tree = ast.parse(code_str)
    cyclic_imports: list[tuple[str, str, str]] = []
    new_body: list[ast.AST] = []

    for node in tree.body:
        # Handle 'from X import Y'
        if (
            isinstance(node, ast.ImportFrom)
            and node.module
            and node.module.startswith(pkg_root)
        ):
            if node.module in cyclic_targets:
                for alias in node.names:
                    cyclic_imports.append(
                        (alias.name, alias.asname or alias.name, node.module)
                    )
                continue  # Skip adding this node to new_body

        # Handle 'import X'
        elif isinstance(node, ast.Import):
            is_fully_cyclic = True
            for alias in node.names:
                if (
                    alias.name.startswith(pkg_root)
                    and alias.name in cyclic_targets
                ):
                    cyclic_imports.append(
                        (
                            alias.name,
                            alias.asname or alias.name.split(".")[-1],
                            alias.name,
                        )
                    )
                else:
                    is_fully_cyclic = False

            if not is_fully_cyclic:
                new_body.append(node)
            continue

        new_body.append(node)

    if not cyclic_imports:
        return code_str

    # Generate the lazy import helper function
    helper_lines = [
        "\n# Lazy imports for cycle breaking",
        "def _resolve_cycles():",
    ]
    for name, alias, mod in cyclic_imports:
        parts = mod.split(".")
        access = (
            f"{'.'.join(parts[:-1])}.{name}"
            if len(parts) > 1
            else f"{mod}.{name}"
        )
        helper_lines.append(f"    import {mod}")
        helper_lines.append(f"    global {alias}")
        helper_lines.append(f"    {alias} = {access}")

    helper_ast = ast.parse("\n".join(helper_lines)).body[0]

    # Reconstruct the AST body
    final_body: list[ast.AST] = []
    inserted = False

    for node in new_body:
        final_body.append(node)
        # Insert helper after the first statement (e.g., after docstring or initial imports)
        if not inserted:
            final_body.append(helper_ast)
            inserted = True

    if not inserted:
        final_body.insert(0, helper_ast)

    # Append the call to resolve cycles at the end of the module
    final_body.append(ast.parse("_resolve_cycles()").body[0])

    tree.body = final_body  # type: ignore
    return ast.unparse(tree)


def consolidate_and_sort_imports(nodes: list[ast.AST]) -> list[ast.AST]:
    """
    Groups, deduplicates, and alphabetically sorts import statements.

    Merges multiple 'from X import Y' statements into a single sorted statement.
    Handles both 'import X' and 'from X import Y', including aliases (asname).
    """
    simple_imports: set[tuple[str, str | None]] = set()

    from_imports: defaultdict[
        tuple[int, str | None], set[tuple[str, str | None]]
    ] = defaultdict(set)

    for node in nodes:
        if isinstance(node, ast.Import):
            for alias in node.names:
                simple_imports.add((alias.name, alias.asname))

        elif isinstance(node, ast.ImportFrom):
            key = (node.level, node.module)
            for alias in node.names:
                from_imports[key].add((alias.name, alias.asname))

    result: list[ast.AST] = []

    for name, asname in sorted(
        simple_imports, key=lambda x: (x[0], x[1] or "")
    ):
        result.append(ast.Import(names=[ast.alias(name=name, asname=asname)]))

    for (level, module), aliases in sorted(
        from_imports.items(), key=lambda x: (x[0][0], x[0][1] or "")
    ):
        sorted_aliases = sorted(aliases, key=lambda x: (x[0], x[1] or ""))

        new_names = [
            ast.alias(name=name, asname=asname)
            for name, asname in sorted_aliases
        ]

        result.append(
            ast.ImportFrom(module=module, names=new_names, level=level)
        )

    return result


def get_build_metadata(pkg_root: str, src_dir: Path) -> dict:
    """
    Gathers build metadata (version, git commit, date, etc.) with safe fallbacks.
    """
    metadata = {
        "version": "dev",
        "commit": "unknown",
        "commit_message": "unknown",
        "build_date": datetime.now(timezone.utc).strftime("%Y-%m-%d"),
    }

    try:
        metadata["version"] = importlib.metadata.version(pkg_root)
    except importlib.metadata.PackageNotFoundError:
        # Fallback: read from pyproject.toml
        pyproject = src_dir.parent / "pyproject.toml"
        if pyproject.exists():
            try:
                content = pyproject.read_text("utf-8")
                match = re.search(
                    r'version\s*=\s*["\']([^"\']+)["\']', content
                )
                if match:
                    metadata["version"] = match.group(1)
            except Exception:
                pass

    try:
        result = subprocess.run(
            ["git", "log", "-1", "--pretty=format:%h|%s"],
            capture_output=True,
            text=True,
            check=True,
            cwd=src_dir,
        )

        parts = result.stdout.strip().split("|", 1)
        if len(parts) == 2:
            metadata["commit"] = parts[0]
            metadata["commit_message"] = parts[1]
        else:
            metadata["commit"] = parts[0] if parts else "unknown"
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass

    return metadata


def append_driver_to_monolith(
    monolith_path: Path | str,
    driver_path: Path | str | None,
    pkg_root: str,
) -> None:
    """
    Appends a driver file to the end of the monolithic bundle.

    Automatically strips:
    1. 'from __future__ import annotations' (already at the top of monolith)
    2. 'from API...' and 'import API...' (API is already flattened into global scope)
    """
    if not driver_path:
        return

    driver_path = Path(driver_path)
    print(f"\nAppending driver: {driver_path.name}")

    if not driver_path.exists():
        print(f" -> Driver file not found: {driver_path}. Skipping.")
        return

    driver_code = driver_path.read_text(encoding="utf-8")
    lines = driver_code.split("\n")
    cleaned_lines = []

    in_multiline_import = False

    import_pattern = re.compile(
        rf"^(?:from|import)\s+{re.escape(pkg_root)}(?:\.|\s|$)"
    )

    print(" -> Processing imports...")
    for line in lines:
        stripped = line.strip()

        if stripped.startswith("from __future__ import"):
            print(" ---> Removing 'from __future__'")
            continue

        if in_multiline_import:
            if stripped.endswith(")"):
                in_multiline_import = False
            continue

        # Detect start of a multiline import from pkg_root
        # Example: "from API.source.core import ("
        if (
            stripped.startswith(f"from {pkg_root}")
            and "(" in stripped
            and not stripped.endswith(")")
        ):
            print(f" ---> Removing multiline 'from {pkg_root}'")
            in_multiline_import = True
            continue

        if import_pattern.match(stripped):
            print(f" ---> Removing 'from {pkg_root}' / 'import {pkg_root}'")
            continue

        cleaned_lines.append(line)

    cleaned_driver = "\n".join(cleaned_lines)

    separator = (
        "\n\n"
        + "# "
        + "=" * 78
        + "\n"
        + f"# DRIVER SECTION (AUTO-APPENDED FROM: {driver_path.name})\n"
        + f"# Original path: {driver_path.absolute()}\n"
        + "# "
        + "=" * 78
        + "\n\n"
    )

    print(" -> Appending monolith...")
    with open(monolith_path, "a", encoding="utf-8") as f:
        f.write(separator)
        f.write(cleaned_driver)

    print("Driver successfully appended to monolith.")


def assemble(src_dir: Path, pkg_root: str, entry: str, out_file: Path) -> None:  # noqa: C901
    """
    Orchestrates the bundling of a Python package into a single monolithic file.

    This is the main entry point for the build process. It executes the following
    pipeline:
      1. Mapping & Parsing: Scans the source directory and parses the AST of all
         modules to extract dependencies, exports, and metadata.
      2. Resolution: Identifies the starting module based on the `entry` point
         specification and traverses the dependency graph to find all reachable modules.
      3. Ordering: Applies topological sorting with heuristic cycle resolution to
         determine the correct execution order.
      4. Deduplication: Collects, deduplicates, and sorts all third-party imports.
      5. Generation: Iterates through the sorted modules, strips docstrings,
         dynamically resolves cyclic imports, and concatenates the code blocks.
      6. Finalization: Prepends a standard header, generates proxy classes for
         import aliases, and writes the final optimized file.

    Args:
        src_dir: The root directory containing the source code.
        pkg_root: The base package name (e.g., 'API').
        entry: The entry point specification (e.g., 'module.path' or 'module.path:Object').
        out_file: The target file path for the generated monolithic bundle.
    """
    print("Scanning package...")
    mod_map = build_mod_map(src_dir, pkg_root)
    parsed_modules: dict[str, ParsedModuleInfo | None] = {
        m: parse_module(m, p, pkg_root, mod_map) for m, p in mod_map.items()
    }

    print(f" -> Resolving entry point '{entry}'...")
    start_mod = find_entry_module(entry, parsed_modules)
    print(f" -> Processing module: {start_mod}")

    print("Collecting dependencies...")
    reachable, graph, aliases = collect_reachable(start_mod, parsed_modules)
    print(f" -> Found {len(reachable)} reachable modules")

    print("Topological sorting + cycle resolution...")
    order, cyclic_nodes = topo_sort_with_cycle_break(graph, reachable)

    print("Final file order:")
    for i, mod in enumerate(order, 1):
        rel = mod_map[mod].relative_to(src_dir)
        dep_names = [
            mod_map[d].relative_to(src_dir).name for d in graph.get(mod, set())
        ]
        deps_str = ", ".join(dep_names) or "none"
        print(f"  {i:3d}. {rel} (depends on: {deps_str})")

    need_future = any(
        p and p["has_future"] for p in parsed_modules.values() if p
    )

    # Collect and deduplicate external imports
    all_ext: list[ast.AST] = []
    for mod in order:
        p = parsed_modules.get(mod)
        if p:
            all_ext.extend(p.get("external_imports", []))

    unique_ext = consolidate_and_sort_imports(all_ext)

    imports_block = (
        "\n".join(ast.unparse(n) for n in unique_ext) + "\n\n"
        if unique_ext
        else ""
    )

    print("\nGenerating monolith...")
    blocks: list[str] = []
    for mod in order:
        p = parsed_modules[mod]
        if not p:
            continue

        # Reconstruct AST without docstrings
        tree = ast.parse("")
        tree.body = p["body"]  # type: ignore
        tree = DocstringRemover().visit(tree)
        ast.fix_missing_locations(tree)
        code = ast.unparse(tree).strip()

        # Break cycles if necessary
        if mod in cyclic_nodes:
            code = break_cyclic_imports_in_code(
                code, set(cyclic_nodes), pkg_root
            )
            print(f" -> Broken cycle in {mod_map[mod].relative_to(src_dir)}")

        if code:
            blocks.append(f"# {mod_map[mod].relative_to(src_dir)}\n{code}")

    # Generate proxy classes for aliases to preserve attribute access
    proxy_lines: list[str] = []
    for alias_name, target_mod in aliases.items():
        if (
            target_mod is None
            or target_mod not in reachable
            or not parsed_modules[target_mod]
        ):
            continue
        pub = parsed_modules[target_mod]["exports"]  # type: ignore
        if not pub:
            continue
        proxy_lines.append(f"\nclass {alias_name}:\n    pass")
        for name in sorted(pub):
            proxy_lines.append(f"    {alias_name}.{name} = {name}")

    # Build final header
    metadata = get_build_metadata(pkg_root, src_dir)

    # Генерация заголовка
    header_lines = []
    if need_future:
        header_lines.append("from __future__ import annotations\n")

    header_lines.append("# " + "=" * 76)
    header_lines.append(f"# AUTO-GENERATED FOR '{entry}' (DO NOT EDIT)")
    header_lines.append(
        f"# Target Class: {entry} | Cython-Ready | Commit: {metadata['commit']}"
    )
    header_lines.append("# " + "=" * 76)
    header_lines.append(f"# Package:      {pkg_root}")
    header_lines.append(f"# Version:      {metadata['version']}")
    header_lines.append(
        f"# Git Commit:   {metadata['commit']} - {metadata['commit_message']}"
    )
    header_lines.append(f"# Entry Point:  {entry}")
    header_lines.append(f"# Build Date:   {metadata['build_date']}")
    header_lines.append(
        f"# Files:        {len(order)} (Strict Dependencies -> Dependents)"
    )
    header_lines.append("# " + "=" * 76 + "\n")

    header = "\n".join(header_lines)

    # Write output
    out_file.write_text(
        header
        + imports_block
        + "\n\n".join(blocks)
        + "\n".join(proxy_lines)
        + "\n",
        "utf-8",
    )

    print(f"\nDone: {out_file}")


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="api-generate-monolith",
        description="Bundle the project into a single monolithic file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python generate_monolith.py ./src my_package main.py\n"
            "  python generate_monolith.py ./src my_package main.py -o ./dist/bundle.py"
        ),
    )

    # Positional arguments (optional due to nargs="?")
    parser.add_argument(
        "src_dir",
        type=Path,
        nargs="?",
        default=find_source_dir(),
        help="Source directory containing the code",
    )
    parser.add_argument(
        "package_root",
        type=str,
        nargs="?",
        default=PACKAGE_ROOT,
        help="Root package name (e.g., 'API.source')",
    )
    parser.add_argument(
        "entry",
        type=str,
        nargs="?",
        default=ENTRY_POINT,
        help="Entry point (e.g., 'main.py' or 'cli:main')",
    )

    # Optional argument (flag)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=OUT_FILE,
        dest="out_file",
        help=f"Path to the output file (default: {OUT_FILE})",
    )

    parser.add_argument(
        "-d",
        "--driver",
        type=Path,
        action="append",  # for --driver a.py --driver b.py
        default=None,
        help="Path to a driver file to append (e.g., RoboDK driver). Can be used multiple times.",
    )

    args = parser.parse_args()

    if not args.src_dir.is_dir():
        parser.error(
            f"Source directory '{args.src_dir.absolute()}' does not "
            f"exist or is not a directory."
        )

    try:
        print(f"Starting build: {args.entry} -> {args.out_file}")
        assemble(args.src_dir, args.package_root, args.entry, args.out_file)
        if args.driver:
            for driver_path in args.driver:
                append_driver_to_monolith(
                    monolith_path=args.out_file,
                    driver_path=driver_path,
                    pkg_root=args.package_root,
                )

        print(f"\nSuccessfully bundled into: {args.out_file.absolute()}")
        print(f"Validate syntax: python -m py_compile {args.out_file}")
        return 0

    except Exception as e:
        print(f"\nCritical build error: {e}", file=sys.stderr)
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
