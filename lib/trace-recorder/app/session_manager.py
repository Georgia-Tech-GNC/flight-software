from enum import Enum
import asyncio
from contextlib import AsyncExitStack
from pathlib import Path
from dataclasses import dataclass

from app.openocd import OpenOCDServer, OpenOCDClient, OpenOCDSWOStream
from app.itm_parser import ITMParser
from app.trace_writer import TraceWriter

SessionState = Enum('SessionState', ['INITIALIZING', 'HALT', 'RUNNING', 'TRACING'])

@dataclass
class SessionConfig:
    board_family: str
    """OpenOCD board family to use, i.e. stm32h7x or stm32f4x"""

    processor_frequency: int
    """CPU core speed in Hz, (the -traceclk flag in OpenOCD)"""

    swo_baudrate: int
    """
    Desired SWO baudrate in Hz. 
    Note that for most targets this is configured dynamically by OpenOCD
    at runtime, not in code or in cubeMX.
    """

    swo_output_port: int = 3443
    """
    Define localhost port used to stream recieved SWO bytes. 
    This number should generally not need to be modified unless you have 
    a conflict on your machine.
    """





class SessionManager:
    """
    Primary application state handler, and also handles
    the creation and deletion of the OpenOCD server and clients
    """

    def __init__(self, openocd_server_args: list[str], session_config: SessionConfig):
        self._config = session_config
        self._server = OpenOCDServer(openocd_server_args)
        self._client = OpenOCDClient()
        self._swo = OpenOCDSWOStream()

        self._lock = asyncio.Lock()        
        self._state = SessionState.INITIALIZING
        self._shutdown = asyncio.Event()

        self._trace_task: asyncio.Task | None = None
        self._trace_writer: TraceWriter | None = None
        self._itm_parser = ITMParser()

    @property
    def state(self) -> SessionState:
        """Current application state"""
        return self._state

    async def reset(self):
        """Reset and halt the core"""
        async with self._lock:
            if self._state == SessionState.INITIALIZING:
                return
            await self._client.reset_halt()
            if self._state == SessionState.TRACING:
                await self.__stop_trace()
            self._itm_parser.reset_buffer()
            self._state = SessionState.HALT

    async def halt(self) -> Path | None:
        """
        Halt the core without resetting.
        Any ongoing trace will be stopped and finalized.

        :returns: Path to the trace file, if one was generated, and None otherwise
        """
        async with self._lock:
            if self._state in [SessionState.INITIALIZING, SessionState.HALT]:
                return
            await self._client.halt()

            trace_path = None
            if self._state == SessionState.TRACING:
                trace_path = await self.__stop_trace()
            self._state = SessionState.HALT
            return trace_path

    async def start_trace(self, trace_path: Path):
        """
        Starts the core and begins a trace

        :param trace_path: Path where the generated trace file should be stored
        """
        async with self._lock:
            if self._state != SessionState.HALT:
                return

            self._trace_writer = TraceWriter(trace_path)
            self._trace_task = asyncio.create_task(self.__trace_loop())

            await self._client.resume()
            self._state = SessionState.TRACING

    async def resume(self):
        """Starts the core without beginning a trace"""
        async with self._lock:
            if self._state != SessionState.HALT:
                return
            await self._client.resume()
            self._state = SessionState.RUNNING

    async def __trace_loop(self):
        """Worker task that handles trace generation"""
        while True:
            if self._trace_writer is None: raise ValueError('TraceWriter was destroyed without killing task')
            data = await self._swo.read()
            list(map(self._trace_writer.accept_packet, self._itm_parser.parse_stream(data)))

    async def __stop_trace(self) -> Path | None:
        """
        Finalizes an in-progress trace and cleans up the writer task.
        Note that this method does not halt the core.

        :returns: The path to the written trace, or None if no trace was written.
        """
        if self._trace_task is not None:
            self._trace_task.cancel()
            await asyncio.gather(self._trace_task, return_exceptions=True)
            self._trace_task = None

        if self._trace_writer is not None:
            trace_path = self._trace_writer.close()
            self._trace_writer = None
            return trace_path
        else:
            return None

    async def __aenter__(self):
        self._shutdown.clear()
        self.__run_task = asyncio.create_task(self.__run())
        return self

    async def __aexit__(self, exc_type, exc, tb):
        self._shutdown.set()
        await self.__run_task

    async def __run(self):
        """
        Background async task that starts all child processes, waits for 
        an exception or a shutdown command, and then cleans up all child 
        processes accordingly.
        """
        async with AsyncExitStack() as exit_stack:
            await self.__connect(exit_stack)
            await self._client.reset_halt()
            self._state = SessionState.HALT

            await self._shutdown.wait()

    async def __connect(self, exit_stack: AsyncExitStack):
        """
        Starts the OpenOCD server and client connections, and adds the
        relevant cleanup steps to the exit stack

        :param exit_stack: In case of failed startup or general server teardown,
        the cleanup steps will be added to this stack
        """
        await exit_stack.enter_async_context(self._server)
        while True:
            if not self._server.running:
                stack_trace = ""
                async with asyncio.timeout(1):
                    stack_trace = "\n".join([item async for item in self._server.output()])
                raise ConnectionError(f"OpenOCD server exited in startup:\n{stack_trace}")
            try:
                await exit_stack.enter_async_context(self._client)
                break
            except OSError:
                await asyncio.sleep(0.1) # Retry connection attempt while server is active
                self._client = OpenOCDClient()

        await self._client.command(
            f"{self._config.board_family}.tpiu configure -protocol uart " +
            f"-traceclk {self._config.processor_frequency} " + 
            f"-pin-freq {self._config.swo_baudrate} " + 
            f"-output :{self._config.swo_output_port}"
        )
        await self._client.command(f"{self._config.board_family}.tpiu enable")
        await self._client.command("itm ports on")
        while True:
            try:
                await exit_stack.enter_async_context(self._swo)
                break
            except OSError:
                await asyncio.sleep(0.1)
                self._swo = OpenOCDSWOStream()
