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

from runners.core import ZephyrBinaryRunner, RunnerCaps, FileType

sys.path.insert(0, os.path.join(os.getcwd(), "modules", "hal", "realtek", "ameba", "scripts",))
import flash

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
