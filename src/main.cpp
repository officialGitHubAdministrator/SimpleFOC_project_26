/*
  MKS DUAL FOC V3.3 PLUS - ABI编码器 + 完整三环闭环位置控制（电机0通道）
  测试库：SimpleFOC 2.1.1
  控制架构：目标角度 -> 位置 P 环 -> 目标速度 -> 速度 PI 环 -> 目标电流 -> 电流 PI 环 -> SVPWM
*/

#include <Arduino.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include <SimpleFOC.h>

// 1. ABI 正交编码器配置：使用 19, 18 引脚，1000线，索引引脚15
//Encoder encoder = Encoder(19, 18, 1000, 15);
Encoder encoder = Encoder(19, 18, 1000);

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// 2. 电机 0 参数与驱动引脚配置（完全匹配你的 MKS DUAL FOC 硬件）
BLDCMotor motor = BLDCMotor(7); // 如果你的电机不是2804(11极对数)，请在此处修改为实际极对数
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22); // 电机0的PWM-A, B, C及使能引脚

// 3. 电流采样配置：对应 MKS DUAL FOC V3.3 PLUS 的板载 INA240A2 方案
// 参数: (修改为 0.001 欧姆采样电阻, 运放放大 50 倍, 采样引脚 GPIO39, GPIO36)
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50.0, 39, 36, _NC);

// 4. 命令解析器
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

uint8_t tx_buf[20];

void setup() {
  Serial.begin(115200);

  // 5. 底层高频中断优化：关闭 ESP32 的无线功能，防止无线中断造成 ABI 编码器丢步
  esp_wifi_stop();
  esp_bt_controller_disable();
  SimpleFOCDebug::enable(&Serial);

  // 6. ABI 编码器硬件初始化
  encoder.quadrature = Quadrature::ON;    // 开启 4 倍频正交解码
  encoder.pullup = Pullup::USE_EXTERN;    // 使用板载外部上拉电阻
  encoder.init();
  encoder.enableInterrupts(doA, doB);     // 绑定 A、B 相外部中断
  motor.linkSensor(&encoder);             // 将编码器关联到电机

  // 7. 驱动器与电流采样联动初始化
  driver.voltage_power_supply = 12;       // 你的供电电压 [V]
  driver.voltage_limit = 9;               // 驱动器限制最大母线电压 9V
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }


  motor.linkDriver(&driver);              // 将驱动器关联到电机

  current_sense.linkDriver(&driver);       // 电流采样绑定驱动器时钟同步
  current_sense.init();                    // 初始化电流采样并校准零点偏置
  motor.linkCurrentSense(&current_sense);  // 将电流采样关联到电机

  // 8. 【核心升级】控制环路配置：切换为完整三环位置闭环
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // 启用高性能 SVPWM 调制
  motor.torque_controller = TorqueControlType::foc_current; // 最内环：开启真正的 d/q 轴电流闭环
  //motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;              // 最外环：配置为角度（位置）控制

  // 9. 串级控制参数调节（位置环与速度环）
  motor.P_angle.P = 0;                     // 角度环 P 参数（决定位置响应灵敏度）
  motor.PID_velocity.P = 0;               // 速度环 P 参数
  motor.PID_velocity.I = 0;                // 速度环 I 参数
  motor.LPF_velocity.Tf = 0.05;             // 速度低通滤波时间常数 [s]

  // 10. 电流环特定的高增益 PID 参数
  motor.PID_current_q.P = 0.05;
  motor.PID_current_q.I = 0;
  motor.PID_current_d.P = 0.05;
  motor.PID_current_d.I = 0;
  motor.LPF_current_q.Tf = 0.005;            // 电流滤波时间常数
  motor.LPF_current_d.Tf = 0.005;

  // 11. 【安全限制】首次位置调试时强烈建议加入以下限制
  motor.velocity_limit = 10;                // 限制最大运动速度为 10 rad/s
  motor.current_limit = 1.5;                // 限制最大力矩电流为 1.5A（防止 PID 调爆时疯狂摆动）

  // 12. 电机与 FOC 初始化
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();                         // 电机自动双向抖动，用于对齐 ABI 零位与电流采样方向

  // 13. 注册串口 T 命令（在串口终端输入如 T3.14，即可控制电机转动到 180 度位置）
  command.add('T', doTarget, "target angle");
  command.add('I', doTarget, "target angle");



  Serial.println("Motor 0 Ready. Set target angle [rad] using 'T' command (e.g. T3.14):");
  _delay(1000);
}


void loop() {
  encoder.update();
  // 核心 FOC 环路：读取 ABI 编码器角度 -> 读取硬件电流 -> Park/Clarke 变换 -> 电流 PID 计算 -> SVPWM 输出
  // 保持高执行频率（>5kHz），发挥最强的电机控噪和控制性能
  motor.loopFOC();

  // 运动控制核心：根据位置环、速度环串级计算，层层下发目标值
  motor.move();

  // 串口通信交互
  command.run();




  static unsigned long last_print_time = 0;

  // 【核心修复 3】：每 20ms 发送一次数据到 VOFA+ (约 50Hz)
  // 绝对不能让串口输出拖慢 loopFOC() 的速度！
  if (millis() - last_print_time > 10) {
    last_print_time = millis(); // 记得把这一行的注释解开，保证定时器正常工作

    float angle = encoder.getAngle();
    float vel   = encoder.getVelocity();

    // 从 motor 结构体中直接提取实时的交轴电流(Iq)和直轴电流(Id)
    float Iq    = motor.current.q;
    float Id    = motor.current.d;
    float Iq_target =motor.target;

    // 依次硬塞进 20 字节数组中
    memcpy(&tx_buf[0],  &angle,     4); // Channel 0
    memcpy(&tx_buf[4],  &vel,       4); // Channel 1
    memcpy(&tx_buf[8],  &Iq,        4); // Channel 2
    memcpy(&tx_buf[12], &Id,        4); // Channel 3
    memcpy(&tx_buf[16], &Iq_target, 4); // Channel 4

    // 一次性发送 20 个字节
    Serial.write(tx_buf, 24);
  }

}


/*
#include <SimpleFOC.h>


// 准备一个能装下 2 个 float(4*2=8字节) + 1 个帧尾(4字节) = 12字节 的数组
uint8_t tx_buf[12];
Encoder encoder = Encoder(19,18,1000,15);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  Serial.begin(115200);
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_EXTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  SimpleFOCDebug::enable(&Serial);
}
void loop() {
  encoder.update(); // 必须有！

  float angle = encoder.getAngle();
  float vel = encoder.getVelocity();
  // 1. 把两个 float 的内存原始数据，硬塞进字节数组的前 8 个位置
  memcpy(&tx_buf[0], &angle, 4);
  memcpy(&tx_buf[4], &vel, 4);

  // 2. 塞入 FireWater 协议规定的 4 字节魔术帧尾
  tx_buf[8]  = 0x00;
  tx_buf[9]  = 0x00;
  tx_buf[10] = 0x80;
  tx_buf[11] = 0x7F; // 注意：有些系统是 7F 80 00 00，VOFA最新版通常小端序是 00 00 80 7F

  // 3. 一次性把这 12 个字节作为二进制流发送出去
  Serial.write(tx_buf, 12);
  delay(10);// 控制在 100Hz 左右的刷新率
}

*/