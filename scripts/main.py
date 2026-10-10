"""Launch a saved quadruped-controller version.

Examples:
    python main.py v1
    python main.py v2
"""

import argparse
from pathlib import Path
import subprocess
import sys


VERSIONS = {
    "v1": "V1.py",
    "v2": "V2.py",
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a saved quadruped controller.")
    parser.add_argument(
        "version",
        nargs="?",
        choices=VERSIONS,
        default="v2",
        help="controller version to run (default: v2)",
    )
    parser.add_argument(
        "script_args",
        nargs=argparse.REMAINDER,
        help="arguments forwarded to the selected version script",
    )
    args = parser.parse_args()

    script = Path(__file__).resolve().parent / "prev_versions" / VERSIONS[args.version]
    return subprocess.run([sys.executable, str(script), *args.script_args], check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
