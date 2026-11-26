import hashlib

import pytest

from mapping.backend import backend_environment


def test_default_backend_environment_is_unchanged_except_explicit_domain():
    original = {"PATH": "original", "ROS_DOMAIN_ID": "0"}
    assert backend_environment({"ros_domain_id": 173}, original) == {
        "PATH": "original",
        "ROS_DOMAIN_ID": "173",
    }
    assert original["ROS_DOMAIN_ID"] == "0"


def test_only_owned_checksum_locked_preload_reaches_child_environment(tmp_path):
    root = tmp_path / "deployment"
    root.mkdir()
    library = root / "origin.so"
    library.write_bytes(b"fixture only, never loaded")
    cfg = dict(
        ros_domain_id=173,
        deployment_root=str(root),
        backend_preload_library=str(library),
        backend_preload_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
    )
    env = backend_environment(cfg, {"PATH": "unchanged"})
    assert env["LD_PRELOAD"] == str(library) and env["PATH"] == "unchanged"
    with pytest.raises(ValueError, match="inherited"):
        backend_environment(cfg, {"LD_PRELOAD": "unvalidated.so"})
    library.write_bytes(b"changed")
    with pytest.raises(ValueError, match="hash"):
        backend_environment(cfg, {})
    cfg["backend_preload_library"] = str(tmp_path / "outside-deployment.so")
    with pytest.raises(ValueError, match="root"):
        backend_environment(cfg, {})
