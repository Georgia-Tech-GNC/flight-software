from dataclasses import dataclass
from enum import Enum, auto

class ITMPacketType(Enum):
    SYNC = auto()
    OVERFLOW = auto()
    RESERVED = auto()
    GLOBAL_TIMESTEP = auto()
    LOCAL_TIMESTEP = auto()
    EXTENSION = auto()
    HARDWARE_SOURCE = auto()
    SOFTWARE_SOURCE = auto()


@dataclass(slots=True)
class ITMPacket:
    """
    Generic dataclass for an ITM packet.

    Non-source packets will always be listed as port -1
    """
    packet_type: ITMPacketType
    port: int
    num_bytes: int
    value: int

class ITMParser:
    """Handles parsing a stream of bytes into ITM packets"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset_buffer(self):
        """Resets the internal buffer"""
        self._buffer.clear()

    def parse_stream(self, data: bytes) -> list[ITMPacket]:
        """
        Adds the provided byte stream to the internal buffer and then parses the stream into ITM packets
        Unused bytes at the end of the stream that may represent an in-progress packet are stored and will
        be added as a prefix onto the next call of this function.

        :param data: Bytes to parse
        :returns: Parsed ITM packets
        """
        self._buffer.extend(data)
        packets: list[ITMPacket] = []

        while len(self._buffer) > 0:
            packet_type, pointer_inc, port = self.__try_parse()

            if pointer_inc == 0: break
            elif packet_type is not None:
                packets.append(ITMPacket(
                    packet_type=packet_type, port=port, num_bytes=pointer_inc-1,
                    value=0x0 if (pointer_inc == 1) else int.from_bytes(self._buffer[1:pointer_inc], 'little')
                ))

            del self._buffer[:pointer_inc]

        return packets

    def __try_parse(self) -> tuple[None | ITMPacketType, int, int]:
        """
        Attempts to parse an ITM packet from the top of the byte buffer.
        This method does NOT modify the buffer.

        This method follows the ARMv7 specification provided here: 
            https://support.arm.com/documentation/ddi0403/d/Appendices/Debug-ITM-and-DWT-Packet-Protocol
        
        This method will attempt to match a prefix of the buffer to a valid ITM packet. 
        If such a matching exists, the type of packet, length, and port will be returned.
        For non-source packets, the port will always be -1.

        If no packet matches a prefix of the buffer, but the buffer itself is a prefix of a 
        valid packet, then the method will return None and a length of 0.

        Otherwise, the method will return None and a length of 1

        .. seealso::
            :meth:`parse_stream`
                Calling function

        :returns: A tuple consisting of the following items:
                    1. The type of the packet parsed, or None if no packet was parsed
                    2. The number of bytes to advance the buffer (which will always equal the
                       length of the packet when a packet is parsed)
                    3. The port specified in the packet, or -1 when not applicable.
        """
        if len(self._buffer) == 0: 
            return None, 0, -1

        header = self._buffer[0]

        if header == 0: # synchronization packet
            return *self.__try_parse_sync_packet(), -1

        elif header & 0b00000011 == 0x00: # Protocol packet
            return *self.__try_parse_protocol_packet(), -1

        else: # Source packet
            payload_size = {0b01: 1, 0b10: 2, 0b11: 4}[header & 0b11]
            source_type = ITMPacketType.HARDWARE_SOURCE if (header & 0b100) else ITMPacketType.SOFTWARE_SOURCE
            packet_channel = header >> 3

            if len(self._buffer) >= payload_size + 1:
                return source_type, payload_size + 1, packet_channel
            else:
                return None, 0, -1 # source packet in progress.

    def __try_identify_protocol_packet(self, exact_size: int | None = None) -> tuple[bool, int]:
        """
        Handles the size identification of a (possible variable length) ARM ITM protocol packets
        from the top of the byte buffer. This method does NOT modify the buffer.

        Protocol packets are defined to start each byte with a continuation bit. 
        This bit is 0 when reading the final byte in the packet and one otherwise.
        Variable length protocol packets may be up to 5 bytes long (including the header), 
        i.e. there may be up to 4 bytes starting with the continuation bit set to 1.
        
        :param exact_size: If specified, only accept packets whose PAYLOAD (not counting metadata)
                            is exactly the specified number of bytes
        :returns: A tuple of the following:
                    1. True if a valid-sized packet was successfully parsed, and false otherwise
                    2. The number of bytes to advance the buffer
        """
        max_len = 5 if exact_size is None else exact_size + 1
        if len(self._buffer) == 0: 
            return False, 0
        if self._buffer[0] & 0x80 == 0:
            return exact_size == None or exact_size == 0, 1 # No payload
        for i in range(1, min(max_len, len(self._buffer))):
            if self._buffer[i] & 0x80 == 0x0: # Continuation bit is 0, so packet ended
                if exact_size is None or i == exact_size: return True, i + 1
                else: return False, 1 # Wrong size
        if len(self._buffer) >= max_len:
            return False, 1 # Packet too long
        else:
            return False, 0 # Packet in progress

    def __try_parse_protocol_packet(self) -> tuple[ITMPacketType | None, int]:
        """
        Attempts to parse a protocol packet from the top of the byte buffer.
        This method does NOT modify the buffer.

        This method follows the ARMv7 specification provided here: 
            https://support.arm.com/documentation/ddi0403/d/Appendices/Debug-ITM-and-DWT-Packet-Protocol/Packet-descriptions/Protocol-packets?lang=en
        
        If the buffer represents a valid prefix of a protocol packet, no packet type will
        be returned and no bytes will be listed to be cleared.

        If no prefix of the buffer represents a valid protocol packet and the buffer itself
        is not a prefix of a valid protocol packet, no packet type will be returned and 
        one byte will be listed to be cleared.

        If a prefix (possibly the entire buffer) represents a valid protocol packet,
        then the length of that prefix will be returned instead, along with the packet type

        .. seealso::
            :meth:`__try_parse`
                Calling function

        :returns: A tuple consisting of the type of packet parsed (or none if no packet was parsed), 
                 followed by the number of bytes to advance the buffer (which is equal to the length
                 of the packet when a packet was parsed)
        """
        if len(self._buffer) == 0: return None, 0
        header = self._buffer[0]

        if header == 0b01110000:
            return ITMPacketType.OVERFLOW, 1

        elif (
            header & 0b10001111 == 0b00000100 or
            header & 0b01111111 == 0b01110000 or
            header & 0b11011111 == 0b10000100 or
            header & 0b11001111 == 0b11000100
        ):
            return ITMPacketType.RESERVED, 1

        elif header == 0b10010100: # global timestamp GTS1 format
            valid, count = self.__try_identify_protocol_packet()
            return ITMPacketType.GLOBAL_TIMESTEP if valid else None, count
            
        elif header == 0b10110100: # global timestamp GTS2 format
            valid, count = self.__try_identify_protocol_packet(4)
            return ITMPacketType.GLOBAL_TIMESTEP if valid else None, count
            
        # Split header into 0bCDDDEEEE
        continuity = header >> 7
        data = (header >> 4) & 0b111
        extra = header & 0xF

        if extra == 0 and ((continuity == 0 and 1 <= data <= 6) or (continuity == 1 and 4 <= data <= 7)):
            # Local timestamp packet
            valid, count = self.__try_identify_protocol_packet()
            return ITMPacketType.LOCAL_TIMESTEP if valid else None, count

        if extra in [0b1000, 0b1100]: # Extension packet
            valid, count = self.__try_identify_protocol_packet()
            return ITMPacketType.EXTENSION if valid else None, count

        else:
            return None, 1 # Unknown protocol packet header

    def __try_parse_sync_packet(self) -> tuple[ITMPacketType | None, int]:
        """
        Attempts to parse a sync packet at the start of the buffer.
        This method does NOT modify the buffer.

        If a prefix of the buffer (including the buffer itself) is a sync packet, 
        this method returns ITMPacketType.SYNC and the length of the packet.
        Alternatively, if the full buffer could be a prefix of a sync packet, this 
        methods returns None and 0
        Otherwise, this method returns None and 1

        .. seealso::
            :meth:`__try_parse`
                Calling function

        :returns: A tuple consisting of the type of packet parsed (which will always be
                  ITMPacketType.Sync) and the number of bytes to advance the buffer (which
                  will be equal to the length of the sync packet when a packet is 
                  successfully parsed).
        """
        zero_bytes = 0
        for i in self._buffer:
            if i != 0: break
            zero_bytes += 1

        if zero_bytes == len(self._buffer):
            # Sync packet is not yet complete
            return None, 0

        elif self._buffer[zero_bytes] == 0x80 and zero_bytes >= 5:
            # Complete sync packet
            return ITMPacketType.SYNC, zero_bytes + 1

        else:
            # Invalid packet
            return None, 1




