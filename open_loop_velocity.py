#
# 日期：2025.8.14 
# 版本：3.0 (增强调试功能)
# 开环速度控制代码
# 电机A2212/15T的极对数：7
#

import machine
import time
import math

# --- 全局常量和变量 ---
poles = 7  # 电机的极对数

# PWM输出引脚定义 (请根据您的ESP32/MCU板子修改)
pwmA_pin = 4
pwmB_pin = 5
pwmC_pin = 6
enable_pin = 40  # 新增：电机使能引脚 GPIO40

voltagePowerSupply = 12.4
open_loop_timestamp = 0
shaft_angle = 0.0  # 机械角度
electrical_angle = 0.0  # 电角度
zero_electric_angle = 0.0
Uq, Ud = 0.0, 0.0
Ualpha, Ubeta = 0.0, 0.0
Ua, Ub, Uc = 0.0, 0.0, 0.0
dc_a, dc_b, dc_c = 0.0, 0.0, 0.0
Ts = 0.0  # 时间间隔

# --- PWM 初始化及使能引脚设置 ---
print("开始初始化设置...")
# 配置使能引脚 GPIO40
enable = machine.Pin(enable_pin, machine.Pin.OUT)
enable.value(1)  # 设置 GPIO40 为高电平以启用电机
print("电机使能引脚 GPIO40 已设置为高电平")

# 初始化 PWM
pwmA = machine.PWM(machine.Pin(pwmA_pin))
pwmB = machine.PWM(machine.Pin(pwmB_pin))
pwmC = machine.PWM(machine.Pin(pwmC_pin))

# 设置 PWM 频率
pwmA.freq(30000)
pwmB.freq(30000)
pwmC.freq(30000)
print("完成PWM初始化设置")
time.sleep(3)

# --- 函数定义 ---
def _electricalAngle(shaft_angle, pole_pairs):
    """计算电角度"""
    return shaft_angle * pole_pairs

def _normalizeAngle(angle):
    """把输入的角度限制在[0, 2pi]"""
    a = math.fmod(angle, 2 * math.pi)
    return a if a >= 0 else a + (2 * math.pi)

def _constrain(value, min_val, max_val):
    """将值限制在 min_val 和 max_val 之间"""
    return max(min_val, min(value, max_val))

def setPwm(Ua, Ub, Uc):
    """根据三相电压设置PWM占空比"""
    global dc_a, dc_b, dc_c
    dc_a = _constrain(Ua / voltagePowerSupply, 0.0, 1.0)
    dc_b = _constrain(Ub / voltagePowerSupply, 0.0, 1.0)
    dc_c = _constrain(Uc / voltagePowerSupply, 0.0, 1.0)
    pwmA.duty_u16(int(dc_a * 65535))
    pwmB.duty_u16(int(dc_b * 65535))
    pwmC.duty_u16(int(dc_c * 65535))

def setPhaseVoltage(p_Uq, p_Ud, p_angle_el):
    """根据 Uq, Ud 和电角度，通过 SVPWM 设置三相电压"""
    global Ualpha, Ubeta, Ua, Ub, Uc
    p_angle_el = _normalizeAngle(p_angle_el + zero_electric_angle)
    Ualpha = -p_Uq * math.sin(p_angle_el)
    Ubeta = p_Uq * math.cos(p_angle_el)
    Ua = Ualpha + voltagePowerSupply / 2
    Ub = (math.sqrt(3) * Ubeta - Ualpha) / 2 + voltagePowerSupply / 2
    Uc = (-Ualpha - math.sqrt(3) * Ubeta) / 2 + voltagePowerSupply / 2
    setPwm(Ua, Ub, Uc)

def velocityOpenloop(target_velocity):
    """开环速度控制，电角度和 Uq 的生成器"""
    global open_loop_timestamp, shaft_angle, electrical_angle, Uq, Ud, Ts
    now_us = time.ticks_us()
    Ts = time.ticks_diff(now_us, open_loop_timestamp) * 1e-6
    if Ts <= 0 or Ts > 0.5:
        Ts = 1e-3
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts)
    electrical_angle = _electricalAngle(shaft_angle, poles)
    Uq = voltagePowerSupply / 10.0
    Ud = 0
    setPhaseVoltage(Uq, Ud, electrical_angle)
    open_loop_timestamp = now_us
    return Uq

def debug():
    """通过 REPL 打印详细的调试信息"""
    print(f"Ts: {Ts:.6f} | "
          f"Shaft Angle: {shaft_angle:.2f} | "
          f"Elec Angle: {electrical_angle:.2f} | "
          f"Uq: {Uq:.2f} | "
          f"Ua: {Ua:.2f} | Ub: {Ub:.2f} | Uc: {Uc:.2f} | "
          f"dc_a: {dc_a:.2f} | dc_b: {dc_b:.2f} | dc_c: {dc_c:.2f}")

# --- 主循环 ---
open_loop_timestamp = time.ticks_us()
debug_counter = 0
debug_interval = 100

while True:
    try:
        velocityOpenloop(10)
        debug_counter += 1
        if debug_counter >= debug_interval:
            debug_counter = 0
            debug()
    except KeyboardInterrupt:
        pwmA.deinit()
        pwmB.deinit()
        pwmC.deinit()
        enable.value(0)  # 关闭电机使能
        print("\n程序已停止，PWM及使能引脚已关闭。")
        break
