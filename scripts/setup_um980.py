#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: setup_um980.py
Description: Backward-compatible wrapper for setup_um98x.py.
Author: Roland L. Guilmet
License: MIT
"""
from __future__ import annotations
import sys
from setup_um98x import main

if __name__ == "__main__":
    sys.exit(main())
