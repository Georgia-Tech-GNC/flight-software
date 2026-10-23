from pathlib import Path

from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical
from textual.screen import ModalScreen
from textual.widgets import Button, Label

class TraceCompleteDialog(ModalScreen[bool]):
    """
    Simple Yes/No Dialog box that asks if the user would 
    like to open a trace in the browser viewer   
    """

    BINDINGS = [
        ("left", "focus_previous"),
        ("right", "focus_next"),
    ]

    def __init__(self, path: Path):
        super().__init__()
        self._path = path

    def compose(self) -> ComposeResult:
        with Vertical(id="dialog"):
            yield Label(f"Trace saved:\n{self._path}")
            yield Label("Open in Perfetto?")

            with Horizontal(id="dialog-controls"):
                yield Button(r"\[Open]", id="open")
                yield Button(r"\[Later]", id="later")

    def on_mount(self) -> None:
        self.query_one("#open", Button).focus()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        self.dismiss(event.button.id == "open")

    def action_focus_previous(self) -> None:
        self.screen.focus_previous()

    def action_focus_next(self) -> None:
        self.screen.focus_next()

    CSS = """
        TraceCompleteDialog {
            align: center middle;
        }

        #dialog {
            width: 60;
            height: auto;
            padding: 1 2;
            border: solid $foreground;
            background: $surface;
        }

        #dialog-controls {
            width: auto;
            height: 1;
            margin-top: 1;
        }

        TraceCompleteDialog Button {
            width: auto;
            min-width: 0;
            height: 1;
            padding: 0 1;
            margin: 0 1 0 0;
            border: none;
            background: transparent;
        }

        TraceCompleteDialog Button:focus {
            text-style: reverse;
        }
    """