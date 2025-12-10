# Copyright (c) 2024 Realtek Semiconductor Corp.
#
# SPDX-License-Identifier: Apache-2.0

'''Zephyr west runner for Ameba.'''

import os
import sys
import argparse
import json
import base64
import shlex
from pathlib import Path
import importlib.util

from runners.core import ZephyrBinaryRunner, RunnerCaps, FileType
from zephyr_ext_common import ZEPHYR_BASE

FLASH_PATH = FLASH_PATH = Path(ZEPHYR_BASE) / "../" / "modules" / "hal" / "realtek" / "ameba" / "scripts" / "flash.py"
mod_name = "realtek_ameba_flash"
spec = importlib.util.spec_from_file_location(mod_name, str(FLASH_PATH))
flash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flash)

class AmebaFlashBinaryRunner(ZephyrBinaryRunner):
    """Runner front-end for AmebaFlash.py."""

    def __init__(self, cfg, args):
        super().__init__(cfg)
        parser = None

        self.args = args

    @classmethod
    def name(cls):
        return 'amebaflash'

    @classmethod
    def capabilities(cls):
        # Enable flash; support erase/reset toggles; flash addr; file override; tool passthrough
        return RunnerCaps(
            commands={'flash'},
            erase=True,
            reset=True
        )

    @classmethod
    def do_add_parser(cls, parser: argparse.ArgumentParser):
        flash.setup_parser(parser)

    @classmethod
    def do_create(cls, cfg, args):
        return AmebaFlashBinaryRunner(cfg, args)

    def do_run(self, command, **kwargs):
        flash.main(self.args)
