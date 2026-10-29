#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: setup_unicore.py
Description: Unified configuration wrapper for Unicore UM980 and UM982 GNSS receivers.
Author: Roland L. Guilmet
License: MIT
"""
from __future__ import annotations
import sys
from setup_um980 import main

if __name__ == "__main__":
    sys.exit(main())
