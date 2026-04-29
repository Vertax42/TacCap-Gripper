"""xense.taccap — TacCap-Gripper Python SDK.

Imported via ``import xense.taccap``. ``xense`` is a PEP 420 namespace
package (no ``__init__.py``), so this subpackage can coexist with any other
``xense.*`` packages installed alongside it.
"""

from ._version import __version__
from . import _taccap_native

hello = _taccap_native.hello
libxense_version = _taccap_native.libxense_version

__all__ = ["__version__", "hello", "libxense_version"]
