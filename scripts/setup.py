import shutil
import subprocess
import sys

from pathlib import Path
from typing import Literal


def on(os: Literal["win32", "linux", "darwin"]):
    return sys.platform.startswith(os)


if __name__ == "__main__":
    if shutil.which("git") is None:
        print("You need to have git installed to run this script", file=sys.stderr)
        exit(-1)

    dest = Path.home()
    # On Windows and Mac arduino uses the Documents folder for the sketch files
    if on("win32") or on("darwin"):
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

    print("Restart Arduino IDE to see the changes")

    if on("win32"):
        print("Make sure also have the drivers installed:")
        print(
            "  https://drive.google.com/file/d/1O-y4VqChv4kQgM-7w4QCXaPL6ap6yvCg/view?usp=sharing"
        )

    if on("darwin"):
        print("Make sure also have the drivers installed:")
        print(
            "https://drive.google.com/file/d/1MI08A74Z41IwNeLIQUV5P7fKNnyCaMa1/view?usp=sharing"
        )
