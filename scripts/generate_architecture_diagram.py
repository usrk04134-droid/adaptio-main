#!/usr/bin/env python3
import os
import re
import subprocess
from collections import defaultdict
from typing import Dict, Tuple, Set, List


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
DOCS_DIR = os.path.join(PROJECT_ROOT, "docs")


INCLUDE_RE = re.compile(r"^\s*#\s*include\s*\"([^\"]+)\"")


def find_source_files(root: str) -> List[str]:
    source_files: List[str] = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith((".cc", ".h", ".hpp")):
                source_files.append(os.path.join(dirpath, filename))
    return source_files


def path_to_module_and_submodule(path: str) -> Tuple[str, str]:
    # Return (module, submodule)
    # Example: src/scanner/joint_tracking/joint_tracking.cc -> (scanner, joint_tracking)
    rel = os.path.relpath(path, SRC_DIR)
    parts = rel.split(os.sep)
    if not parts:
        return ("root", "root")
    # Files directly under src/ should belong to the top-level application group
    if len(parts) == 1:
        return ("app", "core")
    module = parts[0]
    # Group "main" children as their own submodules
    if module in {"common", "scanner", "controller", "configuration", "main"}:
        if len(parts) > 1:
            submodule = parts[1]
        else:
            submodule = "core"
    else:
        # Anything else (benchmarks, tests, etc.)
        submodule = parts[1] if len(parts) > 1 else "core"
    return (module, submodule)


def include_to_module_and_submodule(include_path: str) -> Tuple[str, str]:
    parts = include_path.split("/")
    if not parts:
        return ("", "")
    module = parts[0]
    submodule = parts[1] if len(parts) > 1 else "core"
    return (module, submodule)


def is_project_include(include_path: str) -> bool:
    # We consider project includes to start with one of the known roots
    return include_path.startswith((
        "common/", "scanner/", "controller/", "configuration/", "main/"
    ))


def collect_dependencies() -> Tuple[Set[Tuple[str, str]], Set[Tuple[Tuple[str, str], Tuple[str, str]]]]:
    nodes: Set[Tuple[str, str]] = set()
    edges: Set[Tuple[Tuple[str, str], Tuple[str, str]]] = set()

    for file_path in find_source_files(SRC_DIR):
        module, submodule = path_to_module_and_submodule(file_path)
        nodes.add((module, submodule))
        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    m = INCLUDE_RE.match(line)
                    if not m:
                        continue
                    inc = m.group(1)
                    if not is_project_include(inc):
                        continue
                    imodule, isubmodule = include_to_module_and_submodule(inc)
                    # Ignore self-edges on the same submodule to reduce noise
                    if (module, submodule) == (imodule, isubmodule):
                        continue
                    nodes.add((imodule, isubmodule))
                    edges.add(((module, submodule), (imodule, isubmodule)))
        except Exception:
            # Best effort; skip unreadable files
            pass

    return nodes, edges


def cluster_label(module: str) -> str:
    # Friendly names if desired
    friendly = {
        "common": "common",
        "scanner": "scanner",
        "controller": "controller",
        "configuration": "components",
        "main": "weld",
        "app": "adaptio",
    }
    return friendly.get(module, module)


def generate_dot(nodes: Set[Tuple[str, str]], edges: Set[Tuple[Tuple[str, str], Tuple[str, str]]]) -> str:
    # Group nodes per module
    by_module: Dict[str, Set[str]] = defaultdict(set)
    for module, submodule in nodes:
        by_module[module].add(submodule)

    # Sort for stable output
    modules_sorted = sorted(by_module.keys())

    lines: List[str] = []
    lines.append("digraph Adaptio {\n")
    lines.append("  rankdir=LR;\n")
    lines.append("  graph [fontsize=10, fontname=Helvetica, compound=true, splines=true, overlap=false, ranksep=1, nodesep=0.6];\n")
    lines.append("  node  [shape=box, style=filled, fillcolor=\"#f7f7f7\", color=\"#999999\", fontname=Helvetica, fontsize=10];\n")
    lines.append("  edge  [color=\"#555555\", arrowsize=0.6];\n\n")

    # Clusters
    for idx, module in enumerate(modules_sorted):
        submodules = sorted(by_module[module])
        safe_module = re.sub(r"[^A-Za-z0-9_]", "_", module)
        cluster_name = f"cluster_{safe_module}"
        label = cluster_label(module)
        lines.append(f"  subgraph {cluster_name} {{\n")
        lines.append("    style=rounded;\n")
        lines.append("    color=\"#bbbbbb\";\n")
        lines.append(f"    label=\"{label}\";\n")
        for submodule in submodules:
            node_id = f"{module}__{submodule}"
            pretty_label = submodule.replace('_', ' ')
            lines.append(f"    \"{node_id}\" [label=\"{pretty_label}\"];\n")
        lines.append("  }\n\n")

    # Edges
    for (m_from, s_from), (m_to, s_to) in sorted(edges):
        from_id = f"{m_from}__{s_from}"
        to_id = f"{m_to}__{s_to}"
        if (m_from == m_to) and (s_from == s_to):
            continue
        # Lighter edges within the same cluster
        attrs = "[color=\"#888888\"]" if m_from == m_to else ""
        lines.append(f"  \"{from_id}\" -> \"{to_id}\" {attrs};\n")

    lines.append("}\n")
    return "".join(lines)


def write_and_render(dot_text: str) -> Tuple[str, str]:
    os.makedirs(DOCS_DIR, exist_ok=True)
    dot_path = os.path.join(DOCS_DIR, "architecture.dot")
    svg_path = os.path.join(DOCS_DIR, "architecture.svg")
    with open(dot_path, "w", encoding="utf-8") as f:
        f.write(dot_text)

    # Render using Graphviz dot
    try:
        subprocess.run(
            ["dot", "-Tsvg", dot_path, "-o", svg_path],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as e:
        raise SystemExit("Graphviz 'dot' not found. Please install graphviz.") from e
    return dot_path, svg_path


def main() -> None:
    nodes, edges = collect_dependencies()
    dot_text = generate_dot(nodes, edges)
    dot_path, svg_path = write_and_render(dot_text)
    print(f"Wrote: {dot_path}")
    print(f"Wrote: {svg_path}")


if __name__ == "__main__":
    main()

