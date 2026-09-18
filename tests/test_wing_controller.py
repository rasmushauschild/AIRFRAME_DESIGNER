import struct
import pytest
from airframe_designer.sim.wing_controller import BodyThrustMessage
from airframe_designer.px4.link import mavlink


def test_body_thrust_extension_wire_layout_and_crc():
    m = BodyThrustMessage(1234, 2, 1, [1., 0., 0., 0.], [.25, -.125, -.6])
    packet = m.pack(mavlink.MAVLink(None))
    assert packet[0] == 0xfd
    assert packet[1] == 51
    payload = packet[10:61]
    assert struct.unpack_from('<I', payload)[0] == 1234
    assert payload[36:39] == bytes([2, 1, 39])
    assert struct.unpack_from('<3f', payload, 39) == pytest.approx([.25, -.125, -.6])
    # The older parser ignores the extension but still validates the entire frame CRC.
    parser = mavlink.MAVLink(None)
    decoded = parser.parse_char(packet)
    assert decoded.get_type() == 'SET_ATTITUDE_TARGET'
    assert decoded.type_mask == 39


def test_body_thrust_rejects_mavlink1():
    m = BodyThrustMessage(0, 2, 1, [1., 0., 0., 0.], [0., 0., -.5])
    with pytest.raises(ValueError, match='MAVLink2'):
        m.pack(mavlink.MAVLink(None), force_mavlink1=True)


def test_timeseries_snapshot_remains_aligned_after_recording_continues():
    from airframe_designer.sim.metrics import MetricsRecorder
    recorder = MetricsRecorder()
    recorder.rows.append([0.] * len(recorder.COLS))
    recorder.phase_of_row.append('hover')
    snapshot = recorder.timeseries()
    recorder.rows.append([1.] * len(recorder.COLS))
    recorder.phase_of_row.append('cruise')
    assert len(snapshot['rows']) == len(snapshot['phase']) == 1
    assert snapshot['phase'] == ['hover']
