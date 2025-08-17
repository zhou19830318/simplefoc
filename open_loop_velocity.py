#
# 日期：2025.8.14 
# 版本：4.1 (修复初始化错误)
# 开环速度控制代码 - BLDC电机类
# 电机A2212/15T的极对数：7
#

import machine
import time
import math

class BLDCMotor:
    """无刷直流电机开环控制类"""
    
    def __init__(self, pwm_pins, enable_pin, poles=7, voltage_supply=12.4, pwm_freq=30000):
        """
        初始化BLDC电机控制器
        
        参数:
        pwm_pins: 三相PWM引脚列表 [A相, B相, C相]
        enable_pin: 电机使能引脚
        poles: 电机极对数
        voltage_supply: 供电电压
        pwm_freq: PWM频率
        """
        # 电机参数
        self.poles = poles
        self.voltage_supply = voltage_supply
        self.pwm_freq = pwm_freq
        
        # 引脚配置
        self.pwm_pins = pwm_pins
        self.enable_pin = enable_pin
        
        # 角度和时间变量
        self.shaft_angle = 0.0
        self.electrical_angle = 0.0
        self.zero_electric_angle = 0.0
        self.open_loop_timestamp = 0
        self.Ts = 0.0
        
        # 电压和占空比变量 - 修复这里的初始化
        self.Uq = 0.0
        self.Ud = 0.0
        self.Ualpha = 0.0
        self.Ubeta = 0.0
        self.Ua = 0.0
        self.Ub = 0.0
        self.Uc = 0.0
        self.dc_a = 0.0
        self.dc_b = 0.0
        self.dc_c = 0.0
        
        # 调试相关
        self.debug_counter = 0
        self.debug_interval = 100
        
        # 电机状态
        self.is_enabled = False
        self.is_initialized = False
        
        # 初始化硬件
        self._init_hardware()
    
    def _init_hardware(self):
        """初始化硬件PWM和使能引脚"""
        try:
            print("开始初始化BLDC电机控制器...")
            
            # 配置使能引脚
            self.enable = machine.Pin(self.enable_pin, machine.Pin.OUT)
            self.enable.value(0)  # 初始化时关闭使能
            
            # 初始化PWM
            self.pwmA = machine.PWM(machine.Pin(self.pwm_pins[0]))
            self.pwmB = machine.PWM(machine.Pin(self.pwm_pins[1]))
            self.pwmC = machine.PWM(machine.Pin(self.pwm_pins[2]))
            
            # 设置PWM频率
            self.pwmA.freq(self.pwm_freq)
            self.pwmB.freq(self.pwm_freq)
            self.pwmC.freq(self.pwm_freq)
            
            # 初始化PWM占空比为0
            self.pwmA.duty_u16(0)
            self.pwmB.duty_u16(0)
            self.pwmC.duty_u16(0)
            
            self.is_initialized = True
            print(f"BLDC电机控制器初始化完成")
            print(f"PWM引脚: A={self.pwm_pins[0]}, B={self.pwm_pins[1]}, C={self.pwm_pins[2]}")
            print(f"使能引脚: {self.enable_pin}")
            print(f"极对数: {self.poles}, 供电电压: {self.voltage_supply}V")
            
        except Exception as e:
            print(f"硬件初始化失败: {e}")
            self.is_initialized = False
    
    def enable_motor(self):
        """启用电机"""
        if not self.is_initialized:
            print("错误: 电机未正确初始化")
            return False
        
        self.enable.value(1)
        self.is_enabled = True
        self.open_loop_timestamp = time.ticks_us()
        print("电机已启用")
        return True
    
    def disable_motor(self):
        """禁用电机"""
        if self.is_initialized:
            self.enable.value(0)
            self.pwmA.duty_u16(0)
            self.pwmB.duty_u16(0)
            self.pwmC.duty_u16(0)
        
        self.is_enabled = False
        print("电机已禁用")
    
    def _electrical_angle(self, shaft_angle):
        """计算电角度"""
        return shaft_angle * self.poles
    
    def _normalize_angle(self, angle):
        """将角度归一化到[0, 2π]"""
        a = math.fmod(angle, 2 * math.pi)
        return a if a >= 0 else a + (2 * math.pi)
    
    def _constrain(self, value, min_val, max_val):
        """将值限制在指定范围内"""
        return max(min_val, min(value, max_val))
    
    def _set_pwm(self, Ua, Ub, Uc):
        """设置三相PWM占空比"""
        self.dc_a = self._constrain(Ua / self.voltage_supply, 0.0, 1.0)
        self.dc_b = self._constrain(Ub / self.voltage_supply, 0.0, 1.0)
        self.dc_c = self._constrain(Uc / self.voltage_supply, 0.0, 1.0)
        
        if self.is_enabled:
            self.pwmA.duty_u16(int(self.dc_a * 65535))
            self.pwmB.duty_u16(int(self.dc_b * 65535))
            self.pwmC.duty_u16(int(self.dc_c * 65535))
    
    def _set_phase_voltage(self, Uq, Ud, electrical_angle):
        """通过SVPWM设置三相电压"""
        electrical_angle = self._normalize_angle(electrical_angle + self.zero_electric_angle)
        
        # Clarke变换
        self.Ualpha = -Uq * math.sin(electrical_angle)
        self.Ubeta = Uq * math.cos(electrical_angle)
        
        # 反Clarke变换
        self.Ua = self.Ualpha + self.voltage_supply / 2
        self.Ub = (math.sqrt(3) * self.Ubeta - self.Ualpha) / 2 + self.voltage_supply / 2
        self.Uc = (-self.Ualpha - math.sqrt(3) * self.Ubeta) / 2 + self.voltage_supply / 2
        
        self._set_pwm(self.Ua, self.Ub, self.Uc)
    
    def velocity_openloop(self, target_velocity):
        """
        开环速度控制
        
        参数:
        target_velocity: 目标角速度 (rad/s)
        
        返回:
        当前的Uq值
        """
        if not self.is_enabled:
            return 0.0
        
        # 计算时间间隔
        now_us = time.ticks_us()
        self.Ts = time.ticks_diff(now_us, self.open_loop_timestamp) * 1e-6
        
        # 防止时间间隔异常
        if self.Ts <= 0 or self.Ts > 0.5:
            self.Ts = 1e-3
        
        # 更新角度
        self.shaft_angle = self._normalize_angle(self.shaft_angle + target_velocity * self.Ts)
        self.electrical_angle = self._electrical_angle(self.shaft_angle)
        
        # 设置电压
        self.Uq = self.voltage_supply / 10.0  # 可以根据需要调整
        self.Ud = 0
        
        self._set_phase_voltage(self.Uq, self.Ud, self.electrical_angle)
        
        self.open_loop_timestamp = now_us
        return self.Uq
    
    def set_voltage_limit(self, voltage_limit):
        """设置电压限制"""
        self.voltage_limit = min(voltage_limit, self.voltage_supply)
    
    def debug_print(self):
        """打印调试信息"""
        print(f"Ts: {self.Ts:.6f} | "
              f"Shaft Angle: {self.shaft_angle:.2f} | "
              f"Elec Angle: {self.electrical_angle:.2f} | "
              f"Uq: {self.Uq:.2f} | "
              f"Ua: {self.Ua:.2f} | Ub: {self.Ub:.2f} | Uc: {self.Uc:.2f} | "
              f"dc_a: {self.dc_a:.3f} | dc_b: {self.dc_b:.3f} | dc_c: {self.dc_c:.3f}")
    
    def run_openloop(self, target_velocity, enable_debug=True):
        """
        运行开环控制（带调试输出）
        
        参数:
        target_velocity: 目标角速度
        enable_debug: 是否启用调试输出
        """
        self.velocity_openloop(target_velocity)
        
        if enable_debug:
            self.debug_counter += 1
            if self.debug_counter >= self.debug_interval:
                self.debug_counter = 0
                self.debug_print()
    
    def cleanup(self):
        """清理资源"""
        if self.is_initialized:
            self.disable_motor()
            try:
                self.pwmA.deinit()
                self.pwmB.deinit()
                self.pwmC.deinit()
                print("PWM资源已释放")
            except:
                pass
        print("电机控制器已清理完成")
    
    def get_status(self):
        """获取电机状态信息"""
        return {
            'initialized': self.is_initialized,
            'enabled': self.is_enabled,
            'shaft_angle': self.shaft_angle,
            'electrical_angle': self.electrical_angle,
            'Uq': self.Uq,
            'Ud': self.Ud,
            'Ts': self.Ts
        }


# --- 使用示例 ---
def main():
    """主函数示例"""
    # 创建电机实例
    motor = BLDCMotor(
        pwm_pins=[7, 15, 16],     # PWM引脚 A, B, C
        enable_pin=35,           # 使能引脚
        poles=7,                 # 极对数
        voltage_supply=12.4,     # 供电电压
        pwm_freq=30000          # PWM频率
    )
    
    
    try:
        # 启用电机
        if motor.enable_motor():
            print("开始开环速度控制...")
            time.sleep(2)
            
            # 主控制循环
            while True:
                motor.run_openloop(target_velocity=5)  # 5 rad/s
                time.sleep(0.001)  # 1ms循环周期
                
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止电机...")
    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        motor.cleanup()
        print("程序已安全退出")


if __name__ == "__main__":
    main()
