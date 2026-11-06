"""Pytest anchor so ``g1_action`` core modules import without installation.

Having a conftest at the package root puts this directory on ``sys.path``,
which makes ``import g1_action.<module>`` work when running ``pytest`` (or
``colcon test``) from anywhere. The unit tests only exercise the rclpy-free
core modules, so they run with numpy alone.
"""
