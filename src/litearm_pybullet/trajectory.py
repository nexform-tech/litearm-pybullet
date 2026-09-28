"""Recorded-trajectory types for the simulation.

``JointTrajectory``/``TrajectoryFrame`` are what
:meth:`PyBulletArm.record_trajectory` returns and
:meth:`PyBulletArm.play_trajectory` accepts — a joint path with timing, saved
and loaded as JSON.

Simulation-only: litearm-python has no equivalent. Its motion calls take a target
and block until the arm arrives; a trajectory is only something you can record
and replay here, since the simulation is the thing that can measure a path
frame by frame without a real arm's firmware owning the timing.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional


class TrajectoryFrame:
    """Single frame of a recorded trajectory."""

    def __init__(self, t: float, q: List[float], dq: Optional[List[float]] = None) -> None:
        self.t = t
        self.q = q
        self.dq = dq or [0.0] * len(q)

    def to_dict(self) -> dict:
        return {"t": self.t, "q": self.q, "dq": self.dq}

    @classmethod
    def from_dict(cls, d: dict) -> "TrajectoryFrame":
        return cls(t=d["t"], q=d["q"], dq=d.get("dq"))

    def __repr__(self) -> str:
        return f"TrajectoryFrame(t={self.t:.3f}, q[0]={self.q[0]:.3f})"


class JointTrajectory:
    """A recorded trajectory with frames and metadata."""

    def __init__(
        self,
        frames: List[TrajectoryFrame],
        name: str = "",
        sample_rate_hz: float = 100.0,
        filter_alpha: float = 0.15,
    ) -> None:
        self.frames = frames
        self.name = name
        self.sample_rate_hz = sample_rate_hz
        self.filter_alpha = filter_alpha

    @property
    def q(self) -> List[List[float]]:
        return [f.q for f in self.frames]

    @property
    def t(self) -> List[float]:
        return [f.t for f in self.frames]

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "sample_rate_hz": self.sample_rate_hz,
            "filter_alpha": self.filter_alpha,
            "frames": [f.to_dict() for f in self.frames],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "JointTrajectory":
        return cls(
            frames=[TrajectoryFrame.from_dict(f) for f in d.get("frames", [])],
            name=d.get("name", ""),
            sample_rate_hz=d.get("sample_rate_hz", 100.0),
            filter_alpha=d.get("filter_alpha", 0.15),
        )

    def save(self, path: str) -> None:
        """Save trajectory to a JSON file."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str) -> "JointTrajectory":
        """Load trajectory from a JSON file."""
        d = json.loads(Path(path).read_text())
        return cls.from_dict(d)

    def __repr__(self) -> str:
        return f"JointTrajectory(name={self.name!r}, frames={len(self.frames)})"