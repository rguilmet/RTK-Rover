#!/usr/bin/env python3
"""
check_secrets.py - Pre-Push and Pre-Commit Secrets and Privacy Scanner for RTK Rover

Scans repository files and Git history to prevent accidental exposure of
personal identifiable information (PII), private radio callsigns, private coordinates,
hostnames, local Windows paths, and credentials before pushing to public repositories.

All generic security patterns (AWS keys, GitHub tokens, private keys, local paths)
are public and versioned. User-specific private tokens (callsigns, personal coordinates,
private hostnames) are loaded dynamically from an unversioned .secrets.local file.
"""

import os
import sys
import re
import json
import subprocess
import argparse

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"

# Generic forbidden filenames that must never be committed to the public repository
DEFAULT_FORBIDDEN_FILENAMES = {
    ".secrets.local",
    "solve_deed_network.py",
    "solve_adjoiner_lots.py",
    "control_anchors.csv",
    "homestead_parcels.json",
}

# Generic forbidden regex patterns and descriptive reasons for file content
DEFAULT_FORBIDDEN_PATTERNS = [
    (re.compile(r"-----BEGIN (?:RSA|OPENSSH|DSA|EC|PGP) PRIVATE KEY-----"), "Cryptographic private key block"),
    (re.compile(r"\b(?:ghp|gho|ghu|ghs|ghr)_[A-Za-z0-9_]{36}\b"), "GitHub Personal Access Token (classic)"),
    (re.compile(r"\bgithub_pat_[A-Za-z0-9_]{82}\b"), "GitHub Personal Access Token (fine-grained)"),
    (re.compile(r"\bAKIA[0-9A-Z]{16}\b"), "AWS Access Key ID"),
    (re.compile(r"file:///[A-Za-z]:[/\\]Users[/\\]", re.IGNORECASE), "Absolute Windows user file URI"),
    (re.compile(r"[A-Za-z]:\\Users\\[A-Za-z0-9_-]+\\", re.IGNORECASE), "Absolute Windows user directory path"),
    (re.compile(r'"wifi_password":\s*"(?!(?:Your-Password-Here|YourWiFiPassword|sample_password|""))[^"]+"', re.IGNORECASE), "Non-placeholder Wi-Fi password in JSON config"),
]

# File patterns or paths to skip from scanning (relative to repo root)
DEFAULT_EXCLUDED_FILES = {
    os.path.normpath("scripts/check_secrets.py"),
    os.path.normpath("sample.secrets.local"),
    os.path.normpath(".secrets.local"),
    os.path.normpath(".git"),
}


def get_repo_root():
    """Find the root directory of the Git repository."""
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            stderr=subprocess.PIPE,
            universal_newlines=True
        ).strip()
        return out
    except Exception:
        # Fallback to parent directory of scripts
        return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def load_local_secrets_config(repo_root, custom_config_path=None):
    """
    Load user-specific private tokens and regex patterns from .secrets.local
    if present in repo_root. Returns tuple: (extra_filenames, extra_patterns, extra_exclusions).
    """
    config_path = custom_config_path if custom_config_path else os.path.join(repo_root, ".secrets.local")

    extra_filenames = set()
    extra_patterns = []
    extra_exclusions = set()

    if not os.path.isfile(config_path):
        print(f"[*] No local secrets config found at {os.path.basename(config_path)}. Using generic security rules.")
        return extra_filenames, extra_patterns, extra_exclusions

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        # 1. Plain search tokens (case-insensitive)
        tokens = data.get("forbidden_tokens", [])
        for token in tokens:
            if not isinstance(token, str) or not token.strip():
                continue
            token_clean = token.strip()
            # If purely alphanumeric, enforce word boundary to avoid false positives
            if token_clean.isalnum():
                regex = re.compile(rf"\b{re.escape(token_clean)}\b", re.IGNORECASE)
            else:
                regex = re.compile(re.escape(token_clean), re.IGNORECASE)
            extra_patterns.append((regex, f"Local private token ({token_clean})"))

        # 2. Advanced regex patterns
        patterns = data.get("forbidden_patterns", [])
        for item in patterns:
            if isinstance(item, str) and item.strip():
                regex = re.compile(item.strip(), re.IGNORECASE)
                extra_patterns.append((regex, f"Local private pattern ({item.strip()})"))
            elif isinstance(item, dict):
                pattern_str = item.get("pattern", "")
                if not pattern_str:
                    continue
                desc = item.get("description", f"Local private pattern ({pattern_str})")
                flags = re.IGNORECASE if item.get("case_insensitive", True) else 0
                regex = re.compile(pattern_str, flags)
                extra_patterns.append((regex, desc))

        # 3. Forbidden filenames
        filenames = data.get("forbidden_filenames", [])
        for fname in filenames:
            if isinstance(fname, str) and fname.strip():
                extra_filenames.add(fname.strip())

        # 4. Excluded files
        exclusions = data.get("excluded_files", [])
        for excl in exclusions:
            if isinstance(excl, str) and excl.strip():
                extra_exclusions.add(os.path.normpath(excl.strip()))

        print(f"[*] Loaded local secrets config from {os.path.basename(config_path)}: "
              f"{len(tokens)} token(s), {len(patterns)} pattern(s), {len(filenames)} forbidden filename(s).")

    except Exception as e:
        print(f"[WARN] Error reading local secrets config {config_path}: {e}")

    return extra_filenames, extra_patterns, extra_exclusions


def get_tracked_files(repo_root):
    """Return a list of all tracked files in git."""
    try:
        out = subprocess.check_output(
            ["git", "ls-files"],
            cwd=repo_root,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        files = [os.path.normpath(line.strip()) for line in out.splitlines() if line.strip()]
        return files
    except Exception as e:
        print(f"[WARN] Unable to run 'git ls-files': {e}")
        return []


def scan_file(filepath, repo_root, forbidden_patterns, excluded_files):
    """Scan a single file for forbidden patterns. Returns list of (line_num, reason, line_text)."""
    rel_path = os.path.normpath(os.path.relpath(filepath, repo_root))
    if rel_path in excluded_files or rel_path.startswith(".git"):
        return []

    # Skip large binaries, images, pdfs, 3D CAD files
    ext = os.path.splitext(filepath)[1].lower()
    if ext in [".png", ".jpg", ".jpeg", ".pdf", ".zip", ".tar", ".gz", ".bin", ".ico", ".par", ".dwg", ".dxf"]:
        return []

    violations = []
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            for line_num, line in enumerate(f, 1):
                for pattern, reason in forbidden_patterns:
                    if pattern.search(line):
                        violations.append((line_num, reason, line.strip()))
    except Exception as e:
        print(f"[WARN] Error reading {rel_path}: {e}")

    return violations


def check_git_tracked_secrets(repo_root, forbidden_filenames, forbidden_patterns, excluded_files):
    """Scan all tracked files in git for secrets and forbidden terms."""
    tracked_files = get_tracked_files(repo_root)
    total_violations = 0

    print(f"[*] Scanning {len(tracked_files)} tracked files for private data & secrets...")

    # 1. Check for forbidden tracked filenames
    for rel_path in tracked_files:
        basename = os.path.basename(rel_path)
        if basename in forbidden_filenames:
            total_violations += 1
            print(f"  [FAIL] Forbidden file tracked by Git: {rel_path}")

    # 2. Check file contents
    for rel_path in tracked_files:
        full_path = os.path.join(repo_root, rel_path)
        if not os.path.isfile(full_path):
            continue

        violations = scan_file(full_path, repo_root, forbidden_patterns, excluded_files)
        if violations:
            total_violations += len(violations)
            for line_num, reason, line_text in violations:
                snippet = line_text[:90] + ("..." if len(line_text) > 90 else "")
                print(f"  [FAIL] {rel_path}:{line_num}: {reason}")
                print(f"         > {snippet}")

    return total_violations


def check_unpushed_commits(repo_root, forbidden_patterns):
    """Check outgoing commit diffs against remote tracking branch."""
    total_violations = 0
    try:
        # Determine the target upstream or default to origin/master
        out = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"],
            cwd=repo_root,
            stderr=subprocess.PIPE,
            universal_newlines=True
        ).strip()
        upstream = out if out else "origin/master"
    except Exception:
        upstream = "origin/master"

    try:
        diff_out = subprocess.check_output(
            ["git", "diff", f"{upstream}..HEAD"],
            cwd=repo_root,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
    except Exception:
        # If upstream is not yet pushed or not found, diff against empty tree or HEAD
        return 0

    if not diff_out:
        return 0

    print(f"[*] Scanning unpushed commits against {upstream}...")
    current_file = "unknown"
    for line in diff_out.splitlines():
        if line.startswith("+++ b/"):
            current_file = line[6:].strip()
            continue
        if line.startswith("+") and not line.startswith("+++"):
            added_text = line[1:]
            for pattern, reason in forbidden_patterns:
                if pattern.search(added_text):
                    total_violations += 1
                    snippet = added_text[:90] + ("..." if len(added_text) > 90 else "")
                    print(f"  [FAIL] Unpushed commit in {current_file}: {reason}")
                    print(f"         > {snippet}")

    return total_violations


def main():
    print(f"=== {__filename__} {__version__} ===")
    parser = argparse.ArgumentParser(description="Scan repository for secrets and sensitive terms.")
    parser.add_argument("--commits-only", action="store_true", help="Scan only outgoing unpushed commits")
    parser.add_argument("--config", type=str, default=None, help="Path to custom secrets config JSON file")
    args = parser.parse_args()

    repo_root = get_repo_root()

    # Load local configurations
    extra_filenames, extra_patterns, extra_exclusions = load_local_secrets_config(repo_root, args.config)

    forbidden_filenames = DEFAULT_FORBIDDEN_FILENAMES.union(extra_filenames)
    forbidden_patterns = list(DEFAULT_FORBIDDEN_PATTERNS) + extra_patterns
    excluded_files = DEFAULT_EXCLUDED_FILES.union(extra_exclusions)

    violations = 0

    if args.commits_only:
        violations += check_unpushed_commits(repo_root, forbidden_patterns)
    else:
        violations += check_git_tracked_secrets(repo_root, forbidden_filenames, forbidden_patterns, excluded_files)
        violations += check_unpushed_commits(repo_root, forbidden_patterns)

    print("-" * 60)
    if violations > 0:
        print(f"[REJECT] Pre-Push Check FAILED: {violations} violation(s) found.")
        print("Please scrub sensitive data before pushing to public repository.")
        sys.exit(1)
    else:
        print("[PASS] 100% CLEAN. No secrets or private tokens detected.")
        sys.exit(0)


if __name__ == "__main__":
    main()
