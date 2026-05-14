"""xense.taccap — TacCap-Gripper Python SDK.

Imported via ``import xense.taccap``. ``xense`` is a PEP 420 namespace
package (no ``__init__.py``), so this subpackage can coexist with any other
``xense.*`` packages installed alongside it.

Top-level surface today:

  - Versioning:       ``__version__``, ``libxense_version``, ``hello()``
  - Protocol enums:   ``Address``, ``FrameType``, ``Cmd``, ``ErrorCode``
  - Wire framing:     ``Frame``, ``FrameParser``, ``pack_frame``,
                       ``crc16_modbus``, ``stuff_data``, ``unstuff_data``
  - Serial transport: ``SerialBus``
  - Exceptions:       ``ProtocolError``, ``CrcError``, ``IoError``,
                       ``TimeoutError``
"""

from ._version import __version__
from . import _taccap_native

# ---- Versioning -------------------------------------------------------------
hello = _taccap_native.hello
libxense_version = _taccap_native.libxense_version

# ---- Logging (spdlog-backed, shared with C++ core) --------------------------
log = _taccap_native.log

# ---- Enums ------------------------------------------------------------------
Address = _taccap_native.Address
FrameType = _taccap_native.FrameType
Cmd = _taccap_native.Cmd
ErrorCode = _taccap_native.ErrorCode

# ---- Frame constants --------------------------------------------------------
FRAME_HEAD = _taccap_native.FRAME_HEAD
FRAME_TAIL = _taccap_native.FRAME_TAIL
FRAME_ESCAPE = _taccap_native.FRAME_ESCAPE
MIN_FRAME_LEN = _taccap_native.MIN_FRAME_LEN
MAX_FRAME_LEN = _taccap_native.MAX_FRAME_LEN
MAX_PAYLOAD_LEN = _taccap_native.MAX_PAYLOAD_LEN

# ---- Wire framing -----------------------------------------------------------
Frame = _taccap_native.Frame
FrameParser = _taccap_native.FrameParser
pack_frame = _taccap_native.pack_frame
crc16_modbus = _taccap_native.crc16_modbus
stuff_data = _taccap_native.stuff_data
unstuff_data = _taccap_native.unstuff_data

# ---- Serial transport -------------------------------------------------------
SerialBus = _taccap_native.SerialBus

# ---- Async transport (background reader + ACK matching + DATA dispatch) -----
Transport = _taccap_native.Transport
AckResponse = _taccap_native.AckResponse
TransportStats = _taccap_native.TransportStats

# ---- Components (typed wrappers around Transport + libxense lite) -----------
ImuSample = _taccap_native.ImuSample
EncoderSample = _taccap_native.EncoderSample
MotorStatusSample = _taccap_native.MotorStatusSample
CameraFrame = _taccap_native.CameraFrame
TactileFrame = _taccap_native.TactileFrame
IMU = _taccap_native.IMU
Encoder = _taccap_native.Encoder
Motor = _taccap_native.Motor
Camera = _taccap_native.Camera
TactileSensor = _taccap_native.TactileSensor

# ---- Aggregate gripper + discovery ------------------------------------------
LeaderGripper = _taccap_native.LeaderGripper
FollowerGripper = _taccap_native.FollowerGripper
GripperEndpoints = _taccap_native.GripperEndpoints
Side = _taccap_native.Side
scan_grippers = _taccap_native.scan_grippers
find_one = _taccap_native.find_one
find_left = _taccap_native.find_left
find_right = _taccap_native.find_right

# ---- Exceptions -------------------------------------------------------------
ProtocolError = _taccap_native.ProtocolError
CrcError = _taccap_native.CrcError
IoError = _taccap_native.IoError
TimeoutError = _taccap_native.TimeoutError

__all__ = [
    "__version__",
    "hello",
    "libxense_version",
    # Logging
    "log",
    # Enums
    "Address",
    "FrameType",
    "Cmd",
    "ErrorCode",
    # Frame constants
    "FRAME_HEAD",
    "FRAME_TAIL",
    "FRAME_ESCAPE",
    "MIN_FRAME_LEN",
    "MAX_FRAME_LEN",
    "MAX_PAYLOAD_LEN",
    # Wire framing
    "Frame",
    "FrameParser",
    "pack_frame",
    "crc16_modbus",
    "stuff_data",
    "unstuff_data",
    # Serial transport
    "SerialBus",
    # Async transport
    "Transport",
    "AckResponse",
    "TransportStats",
    # Components
    "ImuSample",
    "EncoderSample",
    "MotorStatusSample",
    "CameraFrame",
    "TactileFrame",
    "IMU",
    "Encoder",
    "Motor",
    "Camera",
    "TactileSensor",
    # Aggregate + discovery
    "LeaderGripper",
    "FollowerGripper",
    "GripperEndpoints",
    "Side",
    "scan_grippers",
    "find_one",
    "find_left",
    "find_right",
    # Exceptions
    "ProtocolError",
    "CrcError",
    "IoError",
    "TimeoutError",
]
