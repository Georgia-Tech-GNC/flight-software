import argparse

from ui import TraceApp
from session_manager import SessionConfig
from pathlib import Path

PRESETS = {
    'ncl-f429': {
        'interface': 'interface/stlink.cfg',
        'target': 'target/stm32f4x.cfg',
        'board_family': 'stm32f4x',
        'processor_frequency': 168000000,
        'swo_baudrate': 1000000,
        'firmware': 'build/ncl-f429-dev/flight-software.elf'
    }
}

def _parse_args():
    parser = argparse.ArgumentParser(
        "trace-recorder", description='CLI tool to collect traces and debug embedded RTOS projecs'
    )

    parser.add_argument('configuration', type=str, choices=PRESETS.keys(), help='Base configuration to use')


if __name__ == "__main__":
    openocd_args = [
        "-f",
        "interface/stlink.cfg",
        "-f",
        "target/stm32f4x.cfg",
    ]

    TraceApp(
        openocd_args=openocd_args,
        session_config=SessionConfig(
            board_family='stm32f4x', processor_frequency=168000000, swo_baudrate=1000000
        ),
        firmware=Path("../../build/ncl-f429-dev/flight-software.elf"),
    ).run()