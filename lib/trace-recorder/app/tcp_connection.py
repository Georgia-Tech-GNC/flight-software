import asyncio

class TCPConnection:
    """Utility class to manage a TCP connection"""

    def __init__(self, host: str, port: int):
        """
        :param host: The host address of the tcp connection
        :param port: The port of the tcp connection
        """
        self._host = host
        self._port = port

        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None

    @property
    def connected(self) -> bool:
        """Returns true if the TCP connection is active"""
        return self._writer is not None and not self._writer.is_closing()

    @property
    def reader(self) -> asyncio.StreamReader:
        """Interface for an asynchronous input stream from the connection"""
        if self._reader is None:
            raise RuntimeError("TCP connection is not open")
        return self._reader

    @property
    def writer(self) -> asyncio.StreamWriter:
        """Interface for an asynchronous output stream to the connection"""
        if self._writer is None:
            raise RuntimeError("TCP connection is not open")
        return self._writer

    async def __connect(self) -> None:
        """Initialize a connectin to the TCP server"""
        if self.connected:
            raise RuntimeError("TCP connection is already open")

        self._reader, self._writer = await asyncio.open_connection(self._host, self._port)

    async def __close(self) -> None:
        """Close a connection to the TCP server"""
        if self._writer is None: return
        self._writer.close()

        try:
            await self._writer.wait_closed()
        except ConnectionError:
            pass

        self._reader = None
        self._writer = None

    async def __aenter__(self):
        await self.__connect()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.__close()