import ast
from pathlib import Path


def imports(directory):
    for path in directory.rglob('*.py'):
        tree = ast.parse(path.read_text(encoding='utf-8-sig'))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                yield from (name.name for name in node.names)
            elif isinstance(node, ast.ImportFrom):
                yield node.module or ''


def test_safety_has_no_communication_dependency():
    root = Path(__file__).resolve().parents[1] / 'src/x3_nav'
    assert not any(name.startswith(('roslibpy', 'x3_nav.bridge', 'x3_nav.nav', 'x3_nav.mission'))
                   for name in imports(root / 'safety'))


def test_vision_does_not_import_execution_or_transport():
    root = Path(__file__).resolve().parents[1] / 'src/x3_nav'
    assert not any(name.startswith(('roslibpy', 'x3_nav.bridge', 'x3_nav.nav', 'x3_nav.mission'))
                   for name in imports(root / 'vision'))
