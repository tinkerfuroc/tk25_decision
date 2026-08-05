from pathlib import Path
from setuptools import find_packages, setup

root = Path(__file__).parent
webui_root = root / "webui"
webui_data_files = []
for directory in sorted({path.parent for path in webui_root.rglob("*") if path.is_file()}):
    files = [str(path.relative_to(root)) for path in sorted(directory.iterdir()) if path.is_file()]
    webui_data_files.append((f"share/gpsr_debug_server/{directory.relative_to(root)}", files))

setup(
    name="gpsr_debug_server",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gpsr_debug_server"]),
        ("share/gpsr_debug_server", ["package.xml"]),
        *webui_data_files,
    ],
    install_requires=["setuptools"],
    extras_require={"web": ["fastapi>=0.100", "uvicorn>=0.30"]},
    entry_points={"console_scripts": ["gpsr-debug-server = gpsr_debug_server.main:main"]},
    zip_safe=False,
)
