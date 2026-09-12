import asyncio
import signal
import os
from collections.abc import AsyncIterator

from tcp_connection import TCPConnection

class OpenOCDSWOStream(TCPConnection):
    """
    Simple OpenOCD client that streams raw binary data recieved over the SWO port.

    Intended to be used as an async context
    """
    def __init__(self, host: str = "127.0.0.1", port: int = 3443):
        """
        :param host: The host address of the openocd output connection
        :param port: The port of the openocd output connection (specified in the openocd 
                     server as --output :xxxx, for example)
        """
        super().__init__(host, port)

    async def read(self, size: int = 65536) -> bytes:
        if self.reader is None:
            raise RuntimeError("SWO stream is not connected")

        data = await self.reader.read(size)
        if not data:
            raise ConnectionError("SWO stream closed")

        return data

class OpenOCDClient(TCPConnection):
    """
    Simple OpenOCD client interface that connects to an active openocd server 
    and facilitates the sending of openocd commands.

    Intended to be used as an async context.
    """
    TERMINATOR = b"\x1a" # All openocd tcl commands should be suffixed with this character

    def __init__(self, host: str = "127.0.0.1", port: int = 6666):
        """
        :param host: The host address of the openocd tcl connection
        :param port: The port of the openocd tcl connection
        """
        super().__init__(host, port)

    async def command(self, command: str) -> str:
        """
        Sends a command to the openocd server. 
        See https://openocd.org/doc/html/General-Commands.html for different commands.

        :param command: The command to send
        """
        if not self.connected or self.reader is None:
            raise RuntimeError("Not connected to OpenOCD")

        self.writer.write(command.encode() + self.TERMINATOR)
        await self.writer.drain()

        response = await self.reader.readuntil(self.TERMINATOR)
        return response[:-1].decode(errors="replace")

    async def halt(self):
        """Utility method that sends an openocd command to halt the processor"""
        await self.command('halt')

    async def resume(self) -> None:
        """Utility method that sends an openocd command to resume the processor"""
        await self.command('resume')

    async def reset_run(self) -> None:
        """Utility method that sends an openocd command to restart the processor"""
        await self.command('reset run')

    async def reset_halt(self) -> None:
        """Utility method that sends an openocd command to restart and then halt the processor"""
        await self.command('reset halt')


class OpenOCDServer:
    """
    Wrapper class for spawning and handling a standalone openocd process.
    Intended to be used as an async context 
    """

    def  __init__(self, args: list[str]):
        """
        :param args: List of arguments to pass to the OpenOCD server on creation
        """
        self._args = args
        self._process: asyncio.subprocess.Process | None = None

    @property
    def running(self) -> bool:
        """Returns true if the openocd server process is currently active"""
        return self._process is not None and self._process.returncode is None
    
    async def __start(self):
        """Start the openocd process"""
        if self.running:
            raise RuntimeError('OpenOCD is already running')

        self._process = await asyncio.create_subprocess_exec(
            "openocd", *self._args, 
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, 
            start_new_session=True
        )

    async def output(self) -> AsyncIterator[str]:
        """Get standard output from the openocd process"""
        if self._process is None or self._process.stdout is None:
            raise RuntimeError("OpenOCD has not been initialized")

        while line := await self._process.stdout.readline():
            yield line.decode(errors='replace').rstrip()

    async def __stop(self):
        """Kill the openocd process, if it is running"""
        if self._process is None: return
        assert self._process is not None

        if self._process.returncode is not None:
            await self._process.wait()
            return

        os.killpg(self._process.pid, signal.SIGTERM)

        try:
            await asyncio.wait_for(self._process.wait(), timeout=2.0)
        except TimeoutError:
            os.killpg(self._process.pid, signal.SIGKILL)
            await self._process.wait()

    async def __aenter__(self) -> 'OpenOCDServer':
        await self.__start()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.__stop()

    async def wait(self) -> int:
        if self._process is None:
            raise RuntimeError("OpenOCD has not been started")

        return await self._process.wait()
