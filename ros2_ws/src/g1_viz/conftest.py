"""Pytest shim: make the in-source package importable without a colcon build."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
