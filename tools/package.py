"""Build the downloadable Fusion add-in archive using only the standard library."""

import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def main():
    root = Path(__file__).resolve().parents[1]
    addin = root / 'FusionFreeDeburr'
    manifest = json.loads((addin / 'FusionFreeDeburr.manifest').read_text(encoding='utf-8'))
    version = manifest['version']
    output = root / 'dist' / f'Fusion-Free-Deburr-v{version}.zip'
    output.parent.mkdir(exist_ok=True)
    with ZipFile(output, 'w', ZIP_DEFLATED) as archive:
        for path in sorted(addin.rglob('*')):
            relative = path.relative_to(addin)
            if any(part.startswith('.') or part == '__pycache__' for part in relative.parts):
                continue
            if path.is_file() and path.suffix not in ('.pyc', '.pyo'):
                archive.write(path, path.relative_to(root))
        for name in ('README.md', 'LICENSE'):
            archive.write(root / name, name)
    print(output)


if __name__ == '__main__':
    main()
