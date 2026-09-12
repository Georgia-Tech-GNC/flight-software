from __future__ import annotations

import subprocess
from contextlib import AsyncExitStack
from pathlib import Path
from datetime import datetime

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Static

from session_manager import SessionManager, SessionState, SessionConfig
from open_trace_in_ui import open_trace
from dialogue_box import TraceCompleteDialog

GDB_STARTUP_COMMANDS = ["target remote localhost:3333", "monitor halt", "tui enable", "layout src"]
PROJECT_PATH = Path(__file__).resolve().parent.parent.resolve()

class TraceApp(App):
    """Main terminal UI manager"""
    
    BINDINGS = [
        ("left", "focus_previous", "Previous"),
        ("right", "focus_next", "Next"),
        ("up", "focus_previous", "Previous"),
        ("down", "focus_next", "Next"),
        ("q", "quit", "Quit"),
        ("ctrl+c", "quit", "Quit application"),
    ]

    def __init__(
        self, openocd_args: list[str], session_config: SessionConfig, firmware: Path, 
        trace_folder: Path = PROJECT_PATH / 'traces', gdb_executable: str = "arm-none-eabi-gdb",
    ):
        """
        :param openocd_args: List of arguments to pass to the OpenOCD server on initialization
        :param session_config: Configuration to provide the application session manager
        :param firmware: Path to the firmware that should be used when running gdb
        :param trace_folder: Path to the folder where recorded traces should be saved
        :param gdb_executable: Path to the gdb installation that should be used when running gdb
        """

        super().__init__()

        self._openocd_args = openocd_args
        self._session_config = session_config
        self._firmware = firmware
        self._gdb_executable = gdb_executable
        self._trace_folder = trace_folder

        self._exit_stack = AsyncExitStack()
        self._session: SessionManager | None = None

    def compose(self) -> ComposeResult:
        """Creates the main UI view"""
        with Vertical(id="main"):
            yield Static("ITM Trace", id="title")

            with Vertical(id="status"):
                yield Static("Target: Connecting...", id="target-status")
                yield Static("State: INITIALIZING", id="state-status")

            with Horizontal(id="controls"):
                yield Button(r"\[Reset]", id="reset")
                yield Button(r"\[Halt]", id="halt")
                yield Button(r"\[Trace]", id="trace")
                yield Button(r"\[Run]", id="run")
                yield Button(r"\[GDB]", id="gdb")

    async def on_mount(self) -> None:
        """Application callback to handle initializing the session manager"""
        await self._exit_stack.__aenter__()
        self._session = await self._exit_stack.enter_async_context(
            SessionManager(self._openocd_args, self._session_config)
        )

        self.query_one("#target-status", Static).update( "Target: Connected")
        self.set_interval(0.2, self._update_state)
        self.query_one("#reset", Button).focus()

    async def on_unmount(self) -> None:
        """Application callback to handle closing the session manager"""
        await self._exit_stack.aclose()

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """Callback to handle UI button presses"""
        if self._session is None: return

        match event.button.id:
            case "reset":
                await self._session.reset()
            case "halt":
                trace_path = await self._session.halt()
                if trace_path is not None:
                    self.push_screen(
                        TraceCompleteDialog(trace_path),
                        lambda open_now: self._handle_trace_dialog(open_now or False, trace_path),
                    )
            case "trace": 
                await self._session.start_trace(self._trace_folder / datetime.now().strftime("trace-%H-%M-%S.pftrace"))
            case "run":
                await self._session.resume()
            case "gdb": 
                await self._enter_gdb()

        self._update_state()

    async def _enter_gdb(self) -> None:
        """Button callback that handles running the gdb server and pausing the UI"""
        if (self._session is None or self._session.state != SessionState.HALT):
            return
        try:
            with self.suspend():
                subprocess.run([
                    self._gdb_executable, str(self._firmware), 
                    *[i for cmd in GDB_STARTUP_COMMANDS for i in ('-ex', cmd)]
                ])
        finally:
            # GDB may have left the target running.
            await self._session.halt()

    def _handle_trace_dialog(self, open_now: bool, path: Path) -> None:
        """
        Helper method that opens the specified trace file in perfetto if open_now is true
        
        :param open_now: If true, the trace file will be opened, otherwise this will be a no-op
        :param path: The path to the trace file to open
        """
        if not open_now: return
        open_trace(path, True, 'https://ui.perfetto.dev')

    def _update_state(self) -> None:
        """Internal helper method that runs periodically and updates the UI from on the session_manager state"""
        if self._session is None: return

        state = self._session.state
        halted = state == SessionState.HALT
        initializing = state == SessionState.INITIALIZING

        self.query_one("#state-status", Static).update(f"State: {state.name}")
        self.query_one("#reset", Button).disabled = initializing
        self.query_one("#halt", Button).disabled = (initializing or halted)
        self.query_one("#trace", Button).disabled = not halted
        self.query_one("#run", Button).disabled = not halted
        self.query_one("#gdb", Button).disabled = not halted

    def action_focus_previous(self) -> None:
        self.screen.focus_previous()

    def action_focus_next(self) -> None:
        self.screen.focus_next()

    CSS = """
        Screen {
            align: center middle;
        }
    
        #main {
            width: 72;
            height: auto;
            padding: 1 2;
            border: solid $foreground;
        }
    
        #title {
            text-style: bold;
            margin-bottom: 1;
        }
    
        #status {
            height: auto;
            margin-bottom: 1;
        }
    
        #controls {
            width: auto;
            height: 1;
        }
    
        Button {
            width: auto;
            min-width: 0;
            height: 1;
            padding: 0 1;
            margin: 0 1 0 0;
            border: none;
            background: transparent;
        }
    
        Button:focus {
            text-style: reverse;
        }
    
        Button:disabled {
            color: $text-disabled;
            text-style: none;
        }
    """
