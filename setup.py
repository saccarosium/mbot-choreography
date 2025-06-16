import shutil
import subprocess
import sys

from pathlib import Path


def panic(msg: str):
    print(msg, file=sys.stderr)
    exit(-1)


if __name__ == "__main__":
    if shutil.which("git") is None:
        panic("You need to have git installed to run this script")

    platform = sys.platform
    dest = Path.home()
    # On Windows and Mac arduino uses the Documents folder for the sketch files
    if platform.startswith("win32") or platform.startswith("darwin"):
        dest = dest / "Documents"
    dest = dest / "Arduino" / "libraries" / "Makeblock"

    print(f"Installing Makeblock libraries in: {dest}")

    subprocess.run(
        [
            "git",
            "clone",
            "--depth=1",
            "https://github.com/Makeblock-official/Makeblock-Libraries.git",
            dest,
        ]
    )
