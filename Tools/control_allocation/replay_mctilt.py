#!/usr/bin/env python3
"""MCTilt 分配器数学回放：冻结日志控制目标，比较旧回中与合力参考。

依赖 numpy、pyulog。只重放分配器，不包含机体动力学、尾流或实际舵机反馈；
不能据此宣称闭环飞行稳定。日志未记录的高频控制输入无法恢复。
运行示例：OPENBLAS_NUM_THREADS=1 python3 Tools/control_allocation/replay_mctilt.py LOG.ulg
"""

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


class Allocator:
    def __init__(self, parameters, initial, hz, revised):
        self.p = dict(parameters)
        self.u = initial.copy()
        self.dt = 1.0 / hz
        self.revised = revised
        self.position = np.array([[self.p[f"CA_ROTOR{i}_P{axis}"] for axis in "XYZ"] for i in range(4)])
        self.ct = np.array([self.p[f"CA_ROTOR{i}_CT"] for i in range(4)])
        self.km = np.array([self.p[f"CA_ROTOR{i}_KM"] for i in range(4)])
        self.mapping = np.array([self.p[f"CA_ROTOR{i}_TILT"] - 1 for i in range(4)], dtype=int)
        self.minimum = np.deg2rad([self.p[f"CA_SV_TL{i}_MINA"] for i in range(4)])
        self.maximum = np.deg2rad([self.p[f"CA_SV_TL{i}_MAXA"] for i in range(4)])
        self.direction = np.deg2rad([self.p[f"CA_SV_TL{i}_TD"] for i in range(4)])
        self.slew = np.array([self.p.get(f"CA_R{i}_SLEW", 0) for i in range(4)]
                             + [self.p.get(f"CA_SV{i}_SLEW", 0) for i in range(4)])
        self.lower = np.r_[np.zeros(4), -np.ones(4)]
        self.upper = np.ones(8)
        self.step_limit = np.r_[np.full(4, .12), np.full(4, .03)]
        self.scale = np.r_[(abs(self.ct) * abs(self.position[:, 1])).sum(),
                           (abs(self.ct) * abs(self.position[:, 0])).sum(),
                           (abs(self.ct) * abs(self.km)).sum(), np.full(3, abs(self.ct).sum())]
        if (self.p["CA_ROTOR_COUNT"] != 4 or self.p["CA_SV_TL_COUNT"] != 4
                or sorted(self.mapping) != list(range(4)) or np.any(self.maximum <= self.minimum)
                or np.any(self.scale < 1e-6) or self.p.get("CA_R_REV", 0)):
            raise ValueError("回放要求四个不可逆电机、四个独立舵机及有效几何")
        if any(self.p[f"CA_SV_TL{i}_CT"] != 3 or self.p[f"CA_SV_TL{i}_AX"] != 0 for i in range(4)):
            raise ValueError("此回放仅支持四个 PitchLike / Yaw and Pitch 舵机")
        self.last_error = np.zeros(6)
        self.reset_filters()

    def reset_filters(self):
        self.filtered = np.zeros(4)
        self.balance_active = False
        self.priority = 0.0

    def angles(self, state):
        return self.minimum + (self.maximum - self.minimum) * (np.clip(state[..., 4:], -1, 1) + 1) / 2

    def wrench(self, state):
        angle = self.angles(state)[..., self.mapping]
        direction = self.direction[self.mapping]
        axis = np.stack([np.sin(angle) * np.cos(direction),
                         np.sin(angle) * np.sin(direction), -np.cos(angle)], axis=-1)
        force = state[..., :4, None] * self.ct[:, None] * axis
        torque = np.cross(self.position, force) - self.km[:, None] * force
        return np.concatenate([torque.sum(axis=-2), force.sum(axis=-2)], axis=-1) / self.scale

    def update(self, demand, balance, roll_priority, reference):
        previous = self.u.copy()
        u = previous.copy()
        target = demand.copy()
        current = self.wrench(u)
        target[4] = current[4]
        u[:4] = np.clip(u[:4] + np.clip(current[5] - target[5], -.12, .12), 0, 1)
        current = self.wrench(u)
        weight = np.array([3., 3., 4., .5, .5, 3.])
        if roll_priority > 0:
            motor_scale = np.clip((.18 - u[:4].min()) / .12, 0, 1)
            tilt_scale = np.clip((abs(self.angles(u)).mean() - np.deg2rad(15)) / np.deg2rad(10), 0, 1)
            shortfall = max(abs(target[0] - current[0]), abs(self.last_error[0]))
            requested = motor_scale * tilt_scale * np.clip((shortfall - .004) / .012, 0, 1)
            self.priority += self.dt / (.2 + self.dt) * (requested - self.priority)
            weight[2] -= 2.25 * np.clip(roll_priority, 0, 1) * self.priority
        else:
            self.priority = 0.0
        perturbed = np.repeat(u[None, :], 8, axis=0) + .01 * np.eye(8)
        jacobian = ((self.wrench(perturbed) - current) / .01).T * weight[:, None]
        inverse = np.linalg.inv(jacobian @ jacobian.T + .15**2 * np.eye(6))
        pinv = jacobian.T @ inverse
        error = target - current
        delta = pinv @ (weight * error)
        secondary = np.zeros(8)
        available = np.max(abs(error[[0, 1, 2, 5]])) < .05
        balance_available = available and balance > 0 and .15 < u[:4].mean() < .85
        if not self.revised:
            balance_available &= np.max(abs(u[4:])) < .88
        if balance_available:
            if self.revised and not self.balance_active:
                self.filtered = u[:4].copy()
            # 旧代码首次进入均衡分支的 dt 为零。
            filter_dt = self.dt if self.revised or self.balance_active else 0.0
            self.filtered += filter_dt / (.35 + filter_dt) * (u[:4] - self.filtered)
            if self.revised:
                reserve = min(.15, .5 * self.filtered.mean())
                spread = np.clip((np.ptp(self.filtered) - .2) / .2, 0, 1)
                secondary[:4] = balance * (spread * (self.filtered.mean() - self.filtered)
                                          + 2 * np.maximum(reserve - self.filtered, 0))
            else:
                spread = np.clip((np.ptp(self.filtered) - .2) / .4, 0, 1)
                secondary[:4] = balance * spread * (self.filtered.mean() - self.filtered)
            self.balance_active = True
        else:
            self.balance_active = False
            self.filtered[:] = 0
        if self.revised and available:
            force = target[3:]
            force_scale = np.clip((np.linalg.norm(force) - .02) / .08, 0, 1)
            horizontal = force[0] * np.cos(self.direction) + force[1] * np.sin(self.direction)
            angle = force_scale * np.arctan2(horizontal, -force[2])
            desired = np.clip(2 * (angle - self.minimum) / (self.maximum - self.minimum) - 1, -1, 1)
            secondary[4:] = reference * (desired - u[4:])
        projected = secondary - pinv @ (jacobian @ secondary)
        scale = min(self.dt, .25 * self.dt / max(np.max(abs(projected)), 1e-8))
        if self.revised and available:
            lower = np.maximum(self.lower - u, -self.step_limit)
            upper = np.minimum(self.upper - u, self.step_limit)
            delta = np.clip(delta, lower, upper)
            room = np.where(projected > 0, upper - delta, delta - lower)
            valid = abs(projected) > np.finfo(np.float32).eps
            if np.any(valid):
                scale = min(scale, np.min(np.maximum(room[valid], 0) / abs(projected[valid])))
        delta += scale * projected
        if not self.revised:
            zero = -1 - 2 * self.minimum / (self.maximum - self.minimum)
            delta[4:] += .002 * (zero - u[4:])
        u = np.clip(u + np.clip(delta, -self.step_limit, self.step_limit), self.lower, self.upper)
        limited = self.slew > 0
        limit = np.divide(self.dt * (self.upper - self.lower), self.slew,
                          out=np.full(8, np.inf), where=limited)
        u = previous + np.clip(u - previous, -limit, limit)
        self.u = u
        self.last_error = demand - self.wrench(u)
        if not np.isfinite(u).all():
            raise RuntimeError("分配产生非有限输出")
        return u


def interpolate(dataset, fields, times):
    return np.column_stack([np.interp(times, dataset["timestamp"] / 1e6, dataset[k]) for k in fields])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--hz", type=int, default=800)
    parser.add_argument("--start", type=float, default=28., help="飞控开机后秒数，不是日志相对时间")
    parser.add_argument("--end", type=float, default=180.)
    parser.add_argument("--reference", type=float, default=.2)
    parser.add_argument("--output", type=Path, default=Path("build/mctilt_replay"))
    args = parser.parse_args()
    ulog = ULog(str(args.log))
    data = lambda name: ulog.get_dataset(name).data
    first = max(data(n)["timestamp"][0] / 1e6 for n in ("vehicle_torque_setpoint", "vehicle_thrust_setpoint", "actuator_servos", "actuator_motors"))
    last = min(data(n)["timestamp"][-1] / 1e6 for n in ("vehicle_torque_setpoint", "vehicle_thrust_setpoint", "actuator_servos", "actuator_motors"))
    start, end = max(first, args.start), min(last, args.end)
    if args.hz <= 0 or end <= start:
        raise ValueError("无效回放时间或频率")
    times = np.arange(start, end, 1 / args.hz)
    target = np.column_stack([interpolate(data(n), [f"xyz[{i}]" for i in range(3)], times)
                              for n in ("vehicle_torque_setpoint", "vehicle_thrust_setpoint")])
    observed = np.column_stack([interpolate(data(n), [f"control[{i}]" for i in range(4)], times)
                                for n in ("actuator_motors", "actuator_servos")])
    parameters = dict(ulog.initial_parameters)
    changes = sorted(ulog.changed_parameters)
    for timestamp, key, value in changes:
        if timestamp / 1e6 <= start:
            parameters[key] = value
    legacy = Allocator(parameters, observed[0], args.hz, False)
    revised = Allocator(parameters, observed[0], args.hz, True)
    changes = [c for c in changes if start < c[0] / 1e6 <= end]
    index = 0
    rows = []
    stride = max(1, args.hz // 10)
    for n, timestamp in enumerate(times):
        while index < len(changes) and changes[index][0] / 1e6 <= timestamp:
            _, key, value = changes[index]
            parameters[key] = value
            if key.startswith(("CA_MCT_", "CA_ROTOR", "CA_SV")):
                # 本日志仅改变均衡与优先级；其他几何改变需重新构造模型，不能静默忽略。
                if key not in ("CA_MCT_MBAL", "CA_MCT_RPRI"):
                    raise ValueError(f"暂不支持回放中改变几何参数：{key}")
                legacy.reset_filters()
                revised.reset_filters()
            index += 1
        gain = parameters.get("CA_MCT_MBAL", 0)
        priority = parameters.get("CA_MCT_RPRI", 0)
        legacy.update(target[n], gain, priority, 0)
        revised.update(target[n], gain, priority, args.reference)
        if n % stride == 0:
            rows.append(np.r_[timestamp, observed[n], legacy.u, revised.u, legacy.last_error, revised.last_error])
    rows = np.array(rows)
    report = {"log": str(args.log), "hz": args.hz, "reference_gain": args.reference,
              "limitation": "Frozen logged commands; mathematical allocator replay, not closed-loop flight simulation. Missing high-rate inputs and physical actuator/aerodynamic errors are not reconstructed.",
              "windows": {}}
    for lo, hi in ((28, 33), (55, 70), (89, 99), (112, 120), (165, 174)):
        mask = (rows[:, 0] >= lo) & (rows[:, 0] < hi)
        if not mask.any():
            continue
        window = {}
        for name, offset, error_offset in (("observed", 1, None), ("legacy", 9, 25), ("revised", 17, 31)):
            state = rows[mask, offset:offset + 8]
            angles = np.rad2deg(revised.angles(state))
            metrics = {"motor_median": np.median(state[:, :4], axis=0).tolist(),
                       "motor_spread_median": float(np.median(np.ptp(state[:, :4], axis=1))),
                       "motor_min_median": float(np.median(state[:, :4].min(axis=1))),
                       "motor_below_012_fraction": float(np.mean(state[:, :4].min(axis=1) < .12)),
                       "servo_spread_deg_median": float(np.median(np.ptp(angles, axis=1)))}
            if error_offset is not None:
                metrics["wrench_error_abs_p95"] = np.percentile(abs(rows[mask, error_offset:error_offset + 6]), 95, axis=0).tolist()
            window[name] = metrics
        report["windows"][f"{lo}-{hi}s"] = window
    args.output.mkdir(parents=True, exist_ok=True)
    header = ["time_s"] + [f"{mode}_u{i}" for mode in ("observed", "legacy", "revised") for i in range(8)]
    header += [f"{mode}_error_{axis}" for mode in ("legacy", "revised") for axis in ("Mx", "My", "Mz", "Fx", "Fy", "Fz")]
    np.savetxt(args.output / "allocation.csv", rows, delimiter=",", header=",".join(header), comments="")
    (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
