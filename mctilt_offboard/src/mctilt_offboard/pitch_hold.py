#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MCTilt 原地变姿测试：固定启动时的当前位置，只改变期望俯仰角。

Position 稳定悬停后启动，预发送目标，再由遥控器切 Offboard。
缓慢进入 pitch_deg → 等待稳定 → 保持 pitch_hold_time → 平滑回正。
全程保持原位置/航向；不自动解锁、切模式、起飞或降落。
异常中止时沿用公共安全逻辑，在当前反馈位置保持并回正。
"""

import math
import rospy

# 共享模块由 catkin_python_setup 安装，避免导入 devel 脚本包装器导致类不可见。
from .figure8_controller import FigureEightOffboard


class PitchHoldOffboard(FigureEightOffboard):
    """复用公共输入链路，但不调用八字轨迹生成器。"""

    def __init__(self):
        super(PitchHoldOffboard, self).__init__()
        self.pitch_hold_time = float(rospy.get_param('~pitch_hold_time', 20.0))
        if not math.isfinite(self.pitch_hold_time) or self.pitch_hold_time <= 0.0:
            raise ValueError('~pitch_hold_time must be finite and positive')
        if not self.control_pitch:
            raise ValueError('Pitch hold test requires ~control_pitch=true')
        if self.pitch_amplitude_deg != 0.0:
            raise ValueError('Pitch hold uses a fixed pitch_deg; set pitch_amplitude_deg=0')

    def _trajectory(self, t, duration):
        """防止未来复用误引入位置运动：任意时刻均返回捕获的固定位置。"""
        return self._hold_reference()

    def _run_trajectory(self):
        """进入倾角由公共 SETTLE 阶段完成；这里只计时保持，结束后公共逻辑回正。"""
        self._trajectory_start = None
        return self._stream_hold_for(self.pitch_hold_time, 'PITCH_HOLD')


def main():
    rospy.init_node('mctilt_offboard', anonymous=False)
    try:
        PitchHoldOffboard().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:
        rospy.logfatal('MCTilt pitch hold failed: %s', exc)
        raise


if __name__ == '__main__':
    main()
