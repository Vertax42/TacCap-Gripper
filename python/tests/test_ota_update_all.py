import importlib.util
import sys
import types
from pathlib import Path


xense = types.ModuleType("xense")
taccap = types.ModuleType("xense.taccap")


class _DummySide:
    Left = "left"
    Right = "right"


class _DummyOtaTargetVersion:
    def __init__(self, major, minor, patch, build=0):
        self.major = major
        self.minor = minor
        self.patch = patch
        self.build = build


def _format_version(major, minor, patch):
    return f"{major}.{minor}.{patch}"


taccap.Side = _DummySide
taccap.LeaderGripper = object
taccap.OtaSession = object
taccap.OtaTargetVersion = _DummyOtaTargetVersion
taccap.crc32_iso_hdlc = lambda data: 0

taccap.log = types.SimpleNamespace(set_level=lambda *args, **kwargs: None)

_calib_flow = types.ModuleType("_calib_flow")
_calib_flow.format_version = _format_version


def _resolve_target(target):
    return None, False, []


_calib_flow.resolve_target = _resolve_target

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "ota_update_example",
    ROOT / "examples" / "ota_update.py",
)

# The stubs exist only so ota_update.py's module-level imports resolve without
# hardware. They MUST be torn down again: pytest imports every test module
# during collection, so a stub left in sys.modules is still there when the
# other test modules run, and they silently get the fake `xense.taccap`
# instead of the real one. That turned a genuine failure in
# test_numpy_views.py into "TypeError: object() takes no arguments" -- an
# error that points at a constructor signature and has nothing to do with the
# actual bug. Anything left installed here would mask future failures too.
_STUBS = {"xense": xense, "xense.taccap": taccap, "_calib_flow": _calib_flow}
_SAVED = {name: sys.modules.get(name) for name in _STUBS}
sys.modules.update(_STUBS)
try:
    mod = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(mod)
finally:
    for name, previous in _SAVED.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


class FakeEndpoint:
    def __init__(self, firmware_sn, side="left", mcu_device="/dev/fake"):
        self.firmware_sn = firmware_sn
        self.side = side
        self.mcu_device = mcu_device
        self.mcu_serial = "fake-serial"


def test_role_upgrade_plan_includes_both_master_and_slave():
    eps_list = [
        FakeEndpoint("TCGU01A28Z0001m", side="left"),
        FakeEndpoint("TCGU01A28Z0002s", side="right"),
    ]

    jobs = mod._build_upgrade_jobs(
        target=None,
        firmware=None,
        all_grippers=True,
        scan_grippers=lambda: eps_list,
    )

    roles = [job["role"] for job in jobs]
    assert roles == ["master", "slave"]
    assert [Path(job["firmware"]).name for job in jobs] == [
        "tc-gu-01-master.bin",
        "tc-gu-01-slave.bin",
    ]


def test_single_slave_role_target_is_supported():
    eps_list = [
        FakeEndpoint("TCGU01A28Z0001m", side="left"),
        FakeEndpoint("TCGU01A28Z0002s", side="right"),
    ]

    jobs = mod._build_upgrade_jobs(
        target="slave",
        firmware=None,
        all_grippers=False,
        scan_grippers=lambda: eps_list,
    )

    assert len(jobs) == 1
    assert jobs[0]["role"] == "slave"
    assert Path(jobs[0]["firmware"]).name == "tc-gu-01-slave.bin"


def test_cli_treats_role_name_as_target_not_firmware_path():
    args = types.SimpleNamespace(
        firmware="slave", target=None, get_status=False, all=False
    )
    normalized = mod._normalize_cli_target(args)
    assert normalized.firmware is None
    assert normalized.target == "slave"
