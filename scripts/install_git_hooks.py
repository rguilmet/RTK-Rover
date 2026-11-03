#!/usr/bin/env python3
"""
install_git_hooks.py - Installs pre-push and pre-commit security verification hooks
into the local .git/hooks directory.
"""

import os
import sys

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"

PRE_PUSH_HOOK_CONTENT = """#!/bin/sh
# Git pre-push hook: Run scripts/check_secrets.py before pushing to remote
if [ -f "scripts/check_secrets.py" ]; then
    echo "[pre-push] Running security and privacy scan..."
    python scripts/check_secrets.py
    RESULT=$?
    if [ $RESULT -ne 0 ]; then
        echo "[pre-push] Push REJECTED: Secrets or private data violations detected."
        echo "[pre-push] Please scrub forbidden patterns or references before pushing."
        exit 1
    fi
fi
exit 0
"""


def main():
    print(f"=== {__filename__} {__version__} ===")
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    hooks_dir = os.path.join(repo_root, ".git", "hooks")

    if not os.path.isdir(hooks_dir):
        print(f"[ERROR] .git/hooks directory not found at {hooks_dir}")
        sys.exit(1)

    pre_push_path = os.path.join(hooks_dir, "pre-push")
    with open(pre_push_path, "w", newline="\n", encoding="utf-8") as f:
        f.write(PRE_PUSH_HOOK_CONTENT)

    # Attempt to make executable if supported
    try:
        os.chmod(pre_push_path, 0o755)
    except Exception:
        pass

    print(f"[OK] Installed pre-push hook: {pre_push_path}")


if __name__ == "__main__":
    main()
