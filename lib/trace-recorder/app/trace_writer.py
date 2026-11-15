from pathlib import Path

from perfetto.trace_builder.proto_builder import StreamingTraceProtoBuilder
from perfetto.protos.perfetto.trace.perfetto_trace_pb2 import TrackEvent

from app.itm_parser import ITMPacket, ITMPacketType

def _decode_str(encoded: int) -> str:
    """
    Decodes the 25-bit compressed string used in NameTaskEvent packets
    
    :param encoded: The encoded value (only the bottom 25 bits are used)
    :returns: The decoded string
    """
    decoded = ''
    for i in range(5):
        raw = encoded & 0x1F
        if 1 <= raw <= 26: decoded += chr(raw - 1 + ord('a'))
        elif raw == 27: decoded += ' '
        elif raw == 28: decoded += '-'
        elif raw == 29: decoded += '_'
        elif raw == 31: decoded += '?'

        encoded >>= 5
    return decoded

class TraceWriter:
    """
    Utility class that handles writing recieved ITM packets to a perfetto trace file  
    """

    SEQUENCE_ID = 1

    def __init__(self, path: str | Path) -> None:
        """
        :param path: The path where the perfetto trace file should be stored
        """
        self._path = path if isinstance(path, Path) else Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(path, 'wb')
        self._builder = StreamingTraceProtoBuilder(self._file)

        self._task_names: dict[int, str] = {}
        self._task_uuids: dict[int, int] = {}
        self._uuid_counter = 0
        self._global_timestamp = 0

    def close(self) -> Path:
        """Path to the written perfetto trace file"""
        self._file.close()
        return self._path


    def accept_packet(self, itm_packet: ITMPacket):
        """
        Adds the provided packet to the trace log.
        Packets are expected to be provided in order.

        :param itm_packet: The packet to add to the file
        """
        if itm_packet.packet_type != ITMPacketType.SOFTWARE_SOURCE: 
            return

        match itm_packet.port:
            case 0:
                self._log_task_name(
                    task_id=(itm_packet.value >> 25) & 0xF, 
                    name=_decode_str(itm_packet.value)
                )
            case 1:
                self._log_task_entered(
                    task_id=(itm_packet.value >> 12) & 0xF,
                    delta_timestamp=itm_packet.value & 0xFFF
                )
            case 2:
                self._log_task_exited(
                    task_id=(itm_packet.value >> 12) & 0xF,
                    delta_timestamp=itm_packet.value & 0xFFF
                )
            case 3:
                self._global_timestamp += itm_packet.value & 0xFFF
            case _:
                return

    def _write_track_event_packet(
            self, uuid: int, 
            timestamp: int | None = None, 
            name: str | None = None, 
            packet_type: TrackEvent.Type | None = None
    ):
        """
        Create a new track event packet with the specifed data and write it to the trace file.
        
        :param uuid: The track id to use
        :param timestamp: Optionally, a timestamp for the event
        :param name: Optionally, a name for the event
        :param packet_type: Optionally, the perfetto track event type to use
        """
        packet = self._builder.create_packet()

        packet.trusted_packet_sequence_id = self.SEQUENCE_ID
        packet.track_event.track_uuid = uuid

        if timestamp is not None:
            packet.timestamp = timestamp
        if packet_type is not None:
            packet.track_event.type = packet_type
        if name is not None:
            packet.track_event.name = name

        self._builder.write_packet(packet)

    def _get_global_timestamp(self, delta_timestamp: int) -> int:
        """
        Increment the internal time tracker and get the next global time
        This is NOT a pure function! 
        It must be called in order, exactly once for each delta timestamp

        :param delta_timestamp: The next delta_timestamp to process (in microseconds)
        :returns: The corresponding global timestamp in nanoseconds
        """
        self._global_timestamp += delta_timestamp
        return self._global_timestamp * 1000

    def _task_uuid(self, task_id: int, name: str | None = None) -> int:
        """
        Gets the track uuid for the provided FreeRTOS task, optionally creating a new 
        perfetto track if one does not already exist.

        :param task_id: The numerical id of the FreeRTOS task to look up
        :param name: Optionally, a human-readable name to assign to the created track.
                     This parameter is ignored if the track already exists.
        :returns: The perfetto track id corresponding to this FreeRTOS task.
        """
        if task_id in self._task_uuids:
            return self._task_uuids[task_id]

        packet = self._builder.create_packet()
        self._uuid_counter += 1

        if name is not None: self._task_names[task_id] = name
        packet.track_descriptor.uuid = self._uuid_counter
        packet.track_descriptor.name = self._task_names.get(task_id, str(task_id))

        self._builder.write_packet(packet)
        self._task_uuids[task_id] = self._uuid_counter
        return self._uuid_counter        

    def _log_task_entered(self, task_id: int, delta_timestamp: int):
        """
        Add a event to the timeline indicating that a FreeRTOS task started running

        :param task_id: The id of the FreeRTOS task that began running, from the target
        :param delta_timestamp: The delta timestamp provided by the target packet 
        """
        self._write_track_event_packet(
            self._task_uuid(task_id), timestamp=self._get_global_timestamp(delta_timestamp),
            name='Running', packet_type=TrackEvent.TYPE_SLICE_BEGIN 
        )

    def _log_task_exited(self, task_id: int, delta_timestamp: int):
        """
        Add a event to the timeline indicating that a FreeRTOS task stopped running

        :param task_id: The id of the FreeRTOS task that stopped running, from the target
        :param delta_timestamp: The delta timestamp provided by the target packet 
        """
        self._write_track_event_packet(
            self._task_uuid(task_id), timestamp=self._get_global_timestamp(delta_timestamp), 
            packet_type=TrackEvent.TYPE_SLICE_END 
        )

    def _log_task_name(self, task_id: int, name: str):
        """
        Specify a human name for the perfetto track associated with this FreeRTOS track.
        This function is a no-op if the task already exists.

        :param task_id: The id of the FreeRTOS task to name
        :param name: The name to assign to this task
        """
        # Trigger initialization for this track with name
        self._task_uuid(task_id, name=name) 