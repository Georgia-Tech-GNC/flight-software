import argparse
from pathlib import Path

from app.ui import TraceApp
from app.session_manager import SessionConfig

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

    # Preset override arguments
    parser.add_argument('--interface', type=str, default=argparse.SUPPRESS)
    parser.add_argument('--target', type=str, default=argparse.SUPPRESS)
    parser.add_argument('--board-family', type=str, default=argparse.SUPPRESS)
    parser.add_argument('--processor-frequency', type=int, default=argparse.SUPPRESS)
    parser.add_argument('--swo-baudrate', type=int, default=argparse.SUPPRESS)
    parser.add_argument('--firmware', type=str, default=argparse.SUPPRESS)

    # Other arguments
    parser.add_argument('--swo-output-port', type=int, default=3443)

    # Apply presets
    args = parser.parse_args()
    preset_args = PRESETS[args.configuration]
    preset_args.update(vars(args))
    return argparse.Namespace(**preset_args)



def main():
    args = _parse_args()
    TraceApp(
        openocd_args=["-f", args.interface, "-f", args.target],
        session_config=SessionConfig(
            board_family=args.board_family, 
            processor_frequency=args.processor_frequency, swo_baudrate=args.swo_baudrate,
            swo_output_port=args.swo_output_port
        ),
        firmware=Path(args.firmware),
    ).run()