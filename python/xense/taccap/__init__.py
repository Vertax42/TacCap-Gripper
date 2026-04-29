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

# ---- Exceptions -------------------------------------------------------------
ProtocolError = _taccap_native.ProtocolError
CrcError = _taccap_native.CrcError
IoError = _taccap_native.IoError
TimeoutError = _taccap_native.TimeoutError

__all__ = [
    "__version__",
    "hello",
    "libxense_version",
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
    # Exceptions
    "ProtocolError",
    "CrcError",
    "IoError",
    "TimeoutError",
]
