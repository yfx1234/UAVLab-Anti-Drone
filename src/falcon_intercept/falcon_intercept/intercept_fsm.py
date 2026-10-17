from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum

from .guidance import GuidanceConfig, compute_approach_velocity


class InterceptState(str, Enum):
    INIT = "INIT"
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    APPROACH = "APPROACH"
    FIRE = "FIRE"
    LAND = "LAND"
    DONE = "DONE"
    ABORT = "ABORT"


@dataclass(frozen=True)
class FsmConfig:
    target_timeout_sec: float = 0.5
    lost_timeout_sec: float = 1.0
    fire_distance_m: float = 2.0
    center_threshold_px: float = 80.0
    fire_hold_frames: int = 3
    max_search_rounds: int = 0
    search_round_period_sec: float = 1.0
    track_confirm_frames: int = 1
    land_hold_sec: float = 1.0
    require_land_confirmation: bool = True
    mission_start_enabled: bool = True
    max_task_time_sec: float = 120.0
    require_center_for_fire: bool = True
    guidance: GuidanceConfig = GuidanceConfig()


@dataclass(frozen=True)
class TargetObservation:
    stamp_s: float
    position_m: tuple[float, float, float]
    velocity_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    front_center_error_px: tuple[float, float] | None = None
    back_center_error_px: tuple[float, float] | None = None


@dataclass(frozen=True)
class FsmOutput:
    state: InterceptState
    state_json: str
    cmd_velocity: tuple[float, float, float] | None
    fire_event_json: str | None
    land_request: bool


def target_distance(position_m: tuple[float, float, float]) -> float:
    x, y, z = position_m
    return math.sqrt(float(x) ** 2 + float(y) ** 2 + float(z) ** 2)


class InterceptFsm:
    def __init__(self, config: FsmConfig) -> None:
        self.config = config
        self.state = InterceptState.INIT
        self._last_target: TargetObservation | None = None
        self._track_frames = 0
        self._fire_hold_frames = 0
        self._fire_sent = False
        self._state_enter_time_s = 0.0
        self._search_miss_rounds = 0
        self._last_search_round_time_s = 0.0
        self._mission_enabled = bool(config.mission_start_enabled)
        self._landed = False
        self._mission_start_time_s: float | None = None

    def set_mission_enabled(self, enabled: bool) -> None:
        self._mission_enabled = bool(enabled)

    def set_landed(self, landed: bool) -> None:
        self._landed = bool(landed)

    def update_target(self, observation: TargetObservation) -> None:
        self._last_target = observation

    def invalidate_target(self) -> None:
        self._last_target = None
        self._track_frames = 0
        self._fire_hold_frames = 0

    def abort(self, now_s: float) -> FsmOutput:
        self._transition(InterceptState.ABORT, now_s)
        return self._output(now_s, reason="manual_abort")

    def step(self, now_s: float) -> FsmOutput:
        now_s = float(now_s)
        if self._mission_start_time_s is None and self._mission_enabled:
            self._mission_start_time_s = now_s
        if (
            self._mission_start_time_s is not None
            and float(self.config.max_task_time_sec) > 0.0
            and now_s - self._mission_start_time_s > float(self.config.max_task_time_sec)
            and self.state not in (InterceptState.LAND, InterceptState.DONE, InterceptState.ABORT)
        ):
            self._transition(InterceptState.ABORT, now_s)
            return self._output(now_s, reason="max_task_time_exceeded")
        target_valid = self._target_is_valid(now_s)

        if self.state == InterceptState.INIT:
            if self._mission_enabled:
                self._transition(InterceptState.SEARCH, now_s)
            else:
                return self._output(now_s, reason="waiting_for_mission_enable")

        if self.state == InterceptState.SEARCH:
            if target_valid:
                self._search_miss_rounds = 0
                self._track_frames = 1
                self._transition(InterceptState.TRACK, now_s)
            else:
                if (
                    now_s - self._last_search_round_time_s
                    >= float(self.config.search_round_period_sec)
                ):
                    self._search_miss_rounds += 1
                    self._last_search_round_time_s = now_s
                    if int(self.config.max_search_rounds) > 0 and self._search_miss_rounds >= int(
                        self.config.max_search_rounds
                    ):
                        self._transition(InterceptState.LAND, now_s)

        elif self.state == InterceptState.TRACK:
            if not target_valid:
                self._track_frames = 0
                self._transition(InterceptState.SEARCH, now_s)
            else:
                self._track_frames += 1
                if self._track_frames >= int(self.config.track_confirm_frames):
                    self._transition(InterceptState.APPROACH, now_s)

        elif self.state == InterceptState.APPROACH:
            if not target_valid:
                self._fire_hold_frames = 0
                self._transition(InterceptState.SEARCH, now_s)
            elif self._fire_window_satisfied():
                self._fire_hold_frames += 1
                if self._fire_hold_frames >= int(self.config.fire_hold_frames):
                    self._transition(InterceptState.FIRE, now_s)
            else:
                self._fire_hold_frames = 0

        elif self.state == InterceptState.FIRE:
            self._transition(InterceptState.LAND, now_s)

        elif self.state == InterceptState.LAND:
            hold_complete = now_s - self._state_enter_time_s >= float(self.config.land_hold_sec)
            if hold_complete and (self._landed or not self.config.require_land_confirmation):
                self._transition(InterceptState.DONE, now_s)

        return self._output(now_s, reason="step")

    def _transition(self, next_state: InterceptState, now_s: float) -> None:
        if self.state == next_state:
            return
        self.state = next_state
        self._state_enter_time_s = float(now_s)
        if next_state == InterceptState.SEARCH:
            self._last_search_round_time_s = float(now_s)
        if next_state != InterceptState.APPROACH:
            self._fire_hold_frames = 0

    def _target_is_valid(self, now_s: float) -> bool:
        if self._last_target is None:
            return False
        if not all(math.isfinite(float(value)) for value in self._last_target.position_m):
            return False
        return 0.0 <= float(now_s) - float(self._last_target.stamp_s) <= float(
            self.config.target_timeout_sec
        )

    def _center_is_valid(self) -> bool:
        if self._last_target is None:
            return False
        if not self.config.require_center_for_fire:
            return True
        errors = [self._last_target.front_center_error_px, self._last_target.back_center_error_px]
        if any(error is None for error in errors):
            return False
        threshold = float(self.config.center_threshold_px)
        return all(abs(error[0]) <= threshold and abs(error[1]) <= threshold for error in errors if error)

    def _fire_window_satisfied(self) -> bool:
        if self._last_target is None:
            return False
        return (
            target_distance(self._last_target.position_m) <= float(self.config.fire_distance_m)
            and self._center_is_valid()
        )

    def _output(self, now_s: float, reason: str) -> FsmOutput:
        target = self._last_target
        distance = None if target is None else target_distance(target.position_m)
        cmd_velocity = None
        if self.state == InterceptState.APPROACH and target is not None:
            cmd_velocity = compute_approach_velocity(target.position_m, self.config.guidance)
        elif self.state in (
            InterceptState.INIT,
            InterceptState.SEARCH,
            InterceptState.TRACK,
            InterceptState.FIRE,
            InterceptState.LAND,
            InterceptState.DONE,
            InterceptState.ABORT,
        ):
            cmd_velocity = (0.0, 0.0, 0.0)

        fire_event = None
        if self.state == InterceptState.FIRE and not self._fire_sent and target is not None:
            fire_event = json.dumps(
                {
                    "event": "FIRE",
                    "distance": distance,
                    "front_error_px": None
                    if target.front_center_error_px is None
                    else target.front_center_error_px[0],
                    "back_error_px": None
                    if target.back_center_error_px is None
                    else target.back_center_error_px[0],
                    "timestamp": now_s,
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self._fire_sent = True

        state_json = json.dumps(
            {
                "state": self.state.value,
                "reason": reason,
                "timestamp": now_s,
                "target_valid": self._target_is_valid(now_s),
                "distance": distance,
                "fire_hold_frames": self._fire_hold_frames,
                "fire_sent": self._fire_sent,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        return FsmOutput(
            state=self.state,
            state_json=state_json,
            cmd_velocity=cmd_velocity,
            fire_event_json=fire_event,
            land_request=self.state == InterceptState.LAND,
        )
