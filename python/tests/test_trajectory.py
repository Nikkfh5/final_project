import csv
import json

from tracker_pkg.domain.pose import Pose
from tracker_pkg.domain.trajectory import Trajectory, TrajectoryPoint


def test_trajectory_save_and_load(tmp_path):
    traj = Trajectory()
    traj.append(TrajectoryPoint(t=0.0, pose=Pose(x=1.0, y=2.0, yaw=0.3), frame=0))
    traj.append(TrajectoryPoint(t=1.0, pose=Pose(x=1.5, y=2.5, yaw=0.4), frame=1))

    csv_path = tmp_path / "traj.csv"
    json_path = tmp_path / "traj.json"

    traj.save_csv(csv_path)
    traj.save_json(json_path)

    # CSV header + 2 rows
    with open(csv_path) as f:
        rows = list(csv.reader(f))
    assert rows[0] == ["t", "x", "y", "theta", "frame"]
    assert len(rows) == 3

    with open(json_path) as f:
        data = json.load(f)
    assert isinstance(data, list)
    assert data[0]["x"] == 1.0
    assert data[1]["frame"] == 1
