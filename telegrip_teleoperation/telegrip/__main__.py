"""
Entry point for running the teleoperation package as a module.
Usage: python -m telegrip [options]
"""

import asyncio
from .main import main_cli

if __name__ == "__main__":
    main_cli() 