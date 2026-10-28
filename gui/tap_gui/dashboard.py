"""Compatibility entry point for the native MuJoCo dashboard."""

from .native_dashboard import NativeDashboard, main, parse_options

__all__ = ["NativeDashboard", "main", "parse_options"]
