"""Forge start/save/stop policy for an independently owned SLAM Toolbox backend."""

import hashlib
import json
import math
import os
import shutil
import time
import uuid
from collections import OrderedDict
from pathlib import Path

from dora import Node
from forge_msgs import PolicyCommand, PolicyCommandStatus

from mapping.map_io import GridSnapshot
from mapping.process import OwnedProcess


def run(cfg: dict) -> None:
    from .backend import materialize

    cfg = materialize(cfg)
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from slam_toolbox.srv import SerializePoseGraph
    from std_srvs.srv import Trigger

    rclpy.init(domain_id=cfg["ros_domain_id"])
    ros = rclpy.create_node("forge_mapping_policy")
    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    _listener = TransformListener(buffer, ros)
    last_snapshot = 0.0
    dora = Node()
    backend = OwnedProcess()
    root = Path(cfg["session_dir"]).resolve()
    archive_root = root / "archives"
    archive_root.mkdir(parents=True, exist_ok=True)
    state = "idle"
    pending = None
    deadline = None
    latest_map = None
    cache = OrderedDict()

    def emit(command, status, message="", outputs=None):
        value = PolicyCommandStatus.from_outputs(
            policy_id="mapping",
            command=command.command,
            request_id=command.request_id,
            status=status,
            message=message,
            outputs=outputs or {},
        )
        dora.send_output("status", value.to_arrow())
        if command.request_id:
            cache[command.request_id] = (
                (command.command, json.dumps(command.inputs(), sort_keys=True)),
                value,
            )
            cache.move_to_end(command.request_id)
            while len(cache) > 128:
                cache.popitem(last=False)

    def runtime(status, message):
        dora.send_output(
            "status",
            PolicyCommandStatus.from_outputs(
                policy_id="mapping",
                command="runtime",
                request_id="",
                status=status,
                message=message,
                outputs={"state": state},
            ).to_arrow(),
        )

    from .snapshot import grid_value, write_snapshot

    snapshot_dir = cfg.get("snapshot_dir")

    def on_map(msg):
        nonlocal latest_map, state, pending
        if snapshot_dir:
            write_snapshot(snapshot_dir, "map", grid_value(msg))
        if state not in ("starting", "running"):
            return
        if msg.info.width * msg.info.height != len(msg.data) or not any(v >= 0 for v in msg.data):
            return
        latest_map = msg
        if state == "starting":
            state = "running"
            emit(
                pending,
                "done",
                "SLAM Toolbox produced a valid map",
                {
                    "ready": True,
                    "frame_id": msg.header.frame_id,
                    "width": msg.info.width,
                    "height": msg.info.height,
                },
            )
            pending = None
            runtime("running", "2D mapping active")

    ros.create_subscription(
        OccupancyGrid,
        "/map",
        on_map,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    graph_client = ros.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
    pcd_client = ros.create_client(Trigger, "/map_save")

    def call(client, request, timeout=20):
        if not client.wait_for_service(timeout_sec=3):
            raise TimeoutError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(ros, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            raise TimeoutError(f"service timeout: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service failed: {client.srv_name}")
        return result

    def save():
        if state != "running" or latest_map is None:
            raise RuntimeError("no running map to save")
        # Capture one immutable grid before spinning ROS service callbacks, which
        # may deliver a newer map. Never create a temporary map subscription.
        snapshot = GridSnapshot.from_message(latest_map)
        save_id = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6]
        final = archive_root / save_id
        folder = archive_root / ("." + save_id + ".pending")
        folder.mkdir()
        try:
            geometry = snapshot.write(folder)
            prefix = folder / "map"
            warnings = []
            pose_graph_saved = pcd_saved = False
            try:
                request = SerializePoseGraph.Request()
                request.filename = str(prefix)
                if call(graph_client, request, timeout=10).result != 0:
                    raise RuntimeError("pose graph serialization failed")
                for ext in (".posegraph", ".data"):
                    if not prefix.with_suffix(ext).is_file():
                        raise RuntimeError("pose graph files missing")
                pose_graph_saved = True
            except Exception as exc:
                warnings.append("pose graph: " + str(exc))
                for ext in (".posegraph", ".data"):
                    prefix.with_suffix(ext).unlink(missing_ok=True)
            try:
                requested_at = time.time()
                response = call(pcd_client, Trigger.Request(), timeout=10)
                cloud = Path(cfg["pcd_path"])
                if (
                    not response.success
                    or not cloud.is_file()
                    or cloud.stat().st_size < 200
                    or cloud.stat().st_mtime < requested_at - 1
                ):
                    raise RuntimeError("fresh PCD map not available")
                with cloud.open("rb") as stream:
                    header = stream.read(1024)
                if b"POINTS 0\n" in header:
                    raise RuntimeError("PCD map is empty")
                shutil.copy2(cloud, folder / "lio_map.pcd")
                pcd_saved = True
            except Exception as exc:
                warnings.append("3D map: " + str(exc))
            for name in ("frame_alignment.json", "trajectory.json"):
                if (root / name).is_file():
                    shutil.copy2(root / name, folder / name)
            manifest = {
                "map_id": cfg["session_id"],
                "version": save_id,
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "yaml_path": str(final / "map.yaml"),
                "image_path": str(final / "map.pgm"),
                "map_frame": "map",
                "grid_saved": True,
                "pose_graph_saved": pose_graph_saved,
                "pcd_saved": pcd_saved,
                "pcd_frame": "camera_init",
                "pcd_loop_closed": False,
                "extrinsic_quality": "initial_reference_direction_checked",
                **geometry,
                "warnings": warnings,
                "files": [],
            }
            for file in sorted(folder.iterdir()):
                with file.open("rb") as stream:
                    digest = hashlib.file_digest(stream, "sha256").hexdigest()
                manifest["files"].append(
                    {"name": file.name, "bytes": file.stat().st_size, "sha256": digest}
                )
            (folder / "manifest.json").write_text(
                json.dumps(manifest, indent=2, ensure_ascii=False)
            )
            folder.rename(final)
        except BaseException:
            shutil.rmtree(folder)
            raise
        return manifest

    runtime("running", "mapping node ready; backend idle")
    try:
        for event in dora:
            if event["type"] == "STOP" or event.get("id") == "finish":
                break
            if event["type"] == "ERROR":
                raise RuntimeError(str(event))
            if event["type"] != "INPUT":
                continue
            if event["id"] == "policy_command":
                command = PolicyCommand.from_arrow(event["value"])
                if command.policy_id != "mapping":
                    continue
                if not command.request_id:
                    emit(command, "rejected", "request_id required")
                    continue
                fingerprint = (command.command, json.dumps(command.inputs(), sort_keys=True))
                if command.request_id in cache:
                    prior, value = cache[command.request_id]
                    if prior == fingerprint:
                        dora.send_output("status", value.to_arrow())
                    else:
                        # Do not replace the original request's idempotency entry.
                        dora.send_output(
                            "status",
                            PolicyCommandStatus.from_outputs(
                                policy_id="mapping",
                                command=command.command,
                                request_id=command.request_id,
                                status="rejected",
                                message="request_id reused with different content",
                                outputs={},
                            ).to_arrow(),
                        )
                    continue
                if command.inputs():
                    emit(command, "rejected", "no input overrides accepted")
                    continue
                if command.command == "start":
                    if state != "idle":
                        emit(command, "rejected", "mapping already active")
                        continue
                    emit(command, "accepted")
                    state, pending, latest_map = "starting", command, None
                    deadline = time.monotonic() + 30
                    ros2 = shutil.which("ros2")
                    if ros2 is None:
                        raise RuntimeError(
                            "ros2 executable unavailable; source the ROS2 setup first"
                        )
                    backend.start(
                        [
                            ros2,
                            "launch",
                            "slam_toolbox",
                            "online_async_launch.py",
                            "slam_params_file:=" + cfg["backend_params"],
                            "use_sim_time:=false",
                        ],
                        cwd=root,
                        log_path=root / "slam_toolbox.log",
                        env={**os.environ, "ROS_DOMAIN_ID": str(cfg["ros_domain_id"])},
                    )
                    emit(command, "running", "awaiting first valid 2D map")
                elif command.command == "save":
                    if state != "running":
                        emit(command, "rejected", "map not ready")
                        continue
                    emit(command, "accepted")
                    try:
                        saved = save()
                        emit(
                            command,
                            "done",
                            "2D map saved; optional exports have warnings"
                            if saved["warnings"]
                            else "map files verified and saved",
                            saved,
                        )
                    except Exception as exc:
                        emit(command, "error", str(exc))
                elif command.command == "stop":
                    emit(command, "accepted")
                    backend.stop()
                    if pending is not None:
                        emit(pending, "error", "stopped before first map")
                        pending = None
                    state = "idle"
                    emit(command, "done", "owned mapping backend stopped", {"stopped": True})
                else:
                    emit(command, "rejected", "unsupported command")
            for _ in range(4):
                rclpy.spin_once(ros, timeout_sec=0)
            if snapshot_dir and time.monotonic() - last_snapshot > 0.2:
                try:
                    tf = buffer.lookup_transform(
                        cfg.get("map_frame", "map"),
                        cfg.get("base_frame", "base_link"),
                        rclpy.time.Time(),
                    )
                    p, q = tf.transform.translation, tf.transform.rotation
                    stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
                    if abs(ros.get_clock().now().nanoseconds / 1e9 - stamp) < 1.5:
                        write_snapshot(
                            snapshot_dir,
                            "pose",
                            {
                                "pose": [p.x, p.y, 2 * math.atan2(q.z, q.w)],
                                "frame_id": cfg.get("map_frame", "map"),
                            },
                        )
                except Exception:
                    pass  # No map transform during backend startup.
                last_snapshot = time.monotonic()
            if state in ("starting", "running") and backend.poll() is not None:
                raise RuntimeError("mapping backend exited unexpectedly")
            if state == "starting" and time.monotonic() > deadline:
                raise TimeoutError("no valid 2D map before timeout")
    except Exception as exc:
        if pending is not None:
            emit(pending, "error", str(exc))
        runtime("error", str(exc))
        raise
    finally:
        backend.stop()
        ros.destroy_node()
        rclpy.shutdown()
