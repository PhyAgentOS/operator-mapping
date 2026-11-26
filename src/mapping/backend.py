"""Materialize optional ROS backend parameter overrides without changing source files."""

import copy
import hashlib
from pathlib import Path

import yaml


def backend_environment(config, environment):
    result = dict(environment)
    result["ROS_DOMAIN_ID"] = str(config["ros_domain_id"])
    if config.get("backend_preload_library"):
        root = Path(config["deployment_root"]).resolve()
        library = Path(config["backend_preload_library"]).resolve()
        if not library.is_relative_to(root) or not library.is_file():
            raise ValueError("backend preload must stay in explicit deployment root")
        if hashlib.sha256(library.read_bytes()).hexdigest() != config.get("backend_preload_sha256"):
            raise ValueError("backend preload library hash differs")
        if result.get("LD_PRELOAD") and result["LD_PRELOAD"] != str(library):
            raise ValueError("unvalidated inherited backend preload")
        result["LD_PRELOAD"] = str(library)
    return result


def materialize(config):
    if "backend_overrides" not in config:
        return config
    values = yaml.safe_load(Path(config["backend_params"]).read_text())

    def merge(target, patch):
        if not isinstance(patch, dict):
            raise ValueError("backend overrides must be a mapping")
        for key, value in patch.items():
            if isinstance(value, dict):
                child = target.setdefault(key, {})
                if not isinstance(child, dict):
                    raise ValueError("cannot merge a mapping into a scalar")
                merge(child, value)
            else:
                target[key] = copy.deepcopy(value)

    merge(values, config["backend_overrides"])
    folder = Path(config["session_dir"])
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / "mapping_backend.yaml"
    target.write_text(yaml.safe_dump(values))
    return {**config, "backend_params": str(target)}
