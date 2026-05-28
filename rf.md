#include <Arduino.h>
#include "esp_wifi.h"     // 引入 ESP-IDF 底层 WiFi 库
#include "esp_bt.h"       // 引入 ESP-IDF 底层蓝牙库

void setup() {
Serial.begin(115200);

// 1. 从操作系统底层彻底注销 WiFi
esp_wifi_stop();        // 停止 WiFi 驱动
esp_wifi_deinit();      // 释放 WiFi 驱动占用的宝贵内存 (SRAM)

// 2. 从硬件控制器层释放蓝牙
esp_bt_controller_disable(); // 彻底关闭蓝牙硬件控制器

Serial.println("\n--- 底层射频彻底释放，SRAM 已回收 ---");

// 接下来写你的 SimpleFOC 初始化代码...
}

void loop() {
// ...
}


/*
#include <Arduino.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include <SimpleFOC.h>


Encoder encoder = Encoder(19,18,1000,15);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22); // set your pins
// 准备一个能装下 2 个 float(4*2=8字节) + 1 个帧尾(4字节) = 12字节 的数组
uint8_t tx_buf[12];


void setup() {
Serial.begin(115200);
esp_wifi_stop();
esp_bt_controller_disable();

encoder.quadrature = Quadrature::ON;
encoder.pullup = Pullup::USE_EXTERN;
encoder.init();
encoder.enableInterrupts(doA, doB);

SimpleFOCDebug::enable(&Serial);
driver.voltage_power_supply = 12;
driver.voltage_limit = 9;

if (!driver.init()){
Serial.println("Driver init failed!");
return;

    // enable driver
    driver.enable();
    Serial.println("Driver ready!");
    _delay(1000);
}
}


float angle0 = 60;
float set_voltage = 9; // Start at 3V


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

// setting pwm
angle0 = fmod(angle0 + 0.01, TWO_PI); // Rotate angle
float pwmA = set_voltage*(sin(angle0) + 1) / 2 ;              // Sinusoidal pattern for testing
float pwmB = set_voltage*(sin(angle0 + TWO_PI/3) + 1) / 2 ;   // Phase shifted by 120 degrees
float pwmC = set_voltage*(sin(angle0 + 2*TWO_PI/3) + 1) / 2 ; // Phase shifted by 240 degrees
driver.setPwm(pwmA, pwmB, pwmC);
delay(100);
}

*/

/*
MKS DUAL FOC 闭环位置控制例程 测试库：SimpleFOC 2.1.1 测试硬件：MKS DUAL FOC V3.1
在串口窗口中输入：T+位置，就可以使得两个电机闭环转动
比如让两个电机都转动180°，则输入其弧度制：T3.14
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 2804云台电机 ，使用自己的电机需要修改PID参数，才能实现更好效果
*/

#include <Arduino.h>
#include "esp_wifi.h"     // 引入 ESP-IDF 底层 WiFi 库 (用于降温)
#include "esp_bt.h"       // 引入 ESP-IDF 底层蓝牙库
#include <SimpleFOC.h>

// =========================================================================
// 1. 编码器实例化与中断函数定义 (ABI 接口)
// =========================================================================

// --- 0号接口 (连接 Yaw轴 3510 电机) ---
Encoder encoder0 = Encoder(19, 18, 1000, 15); // A0=19, B0=18, Z0=15
void doA0(){ encoder0.handleA(); }
void doB0(){ encoder0.handleB(); }

// --- 1号接口 (连接 Pitch轴 C2208 电机) ---
Encoder encoder1 = Encoder(23, 5, 1000, 13);  // A1=23, B1=5, Z1=13
void doA1(){ encoder1.handleA(); }
void doB1(){ encoder1.handleB(); }

// =========================================================================
// 2. 电机与驱动器实例化
// =========================================================================

// --- 电机0 (Yaw轴) ---
// 【注意】：如果你用了 3510 电机，极对数必须填 11！如果还是 2804，请改回 7！
BLDCMotor motor0 = BLDCMotor(7);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32, 33, 25, 22);

// --- 电机1 (Pitch轴) ---
// C2208 极对数是 7
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

// =========================================================================
// 3. 串口指令接收器
// =========================================================================
float target_angle = 0; // 目标角度 (弧度)
Commander command = Commander(Serial);
void doTarget(char* cmd) {
command.scalar(&target_angle, cmd);

// 加上这两句，如果收到了，它一定会打印出来！
Serial.print("收到指令，更新角度为: ");
Serial.println(target_angle);
}
void setup() {
Serial.begin(115200);

// 核心降温：彻底关闭射频，释放内存
esp_wifi_stop();
esp_bt_controller_disable();

// --------------------------------------------------
// 初始化 0 号编码器并绑定中断
// --------------------------------------------------
encoder0.quadrature = Quadrature::ON;
encoder0.pullup = Pullup::USE_EXTERN;
encoder0.init();
encoder0.enableInterrupts(doA0, doB0);
motor0.linkSensor(&encoder0); // 将编码器0绑定给电机0

// --------------------------------------------------
// 初始化 1 号编码器并绑定中断
// --------------------------------------------------
encoder1.quadrature = Quadrature::ON;
encoder1.pullup = Pullup::USE_EXTERN;
encoder1.init();
encoder1.enableInterrupts(doA1, doB1);
motor1.linkSensor(&encoder1); // 将编码器1绑定给电机1

// --------------------------------------------------
// 初始化驱动器
// --------------------------------------------------
driver0.voltage_power_supply = 12;
driver0.init();
motor0.linkDriver(&driver0);

driver1.voltage_power_supply = 12;
driver1.init();
motor1.linkDriver(&driver1);

// --------------------------------------------------
// 控制模式与 PID 参数设置
// --------------------------------------------------
motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

motor0.controller = MotionControlType::angle;
motor1.controller = MotionControlType::angle;

// 速度 PI 环设置 (硬度与阻尼)
motor0.PID_velocity.P = 0.1;
motor0.PID_velocity.I = 1.0;
motor0.LPF_velocity.Tf = 0.01;

motor1.PID_velocity.P = 0.1;
motor1.PID_velocity.I = 1.0;
motor1.LPF_velocity.Tf = 0.01;

// 角度 P 环设置 (追踪敏捷度)
motor0.P_angle.P = 20;
motor1.P_angle.P = 20;

// 【保命限制】：电压上限，建议设为 3V~5V，不要设1V(力气太小)或12V(会烧)
motor0.voltage_limit = 5.0;
motor1.voltage_limit = 5.0;

// 速度限制 [rad/s]
motor0.velocity_limit = 20;
motor1.velocity_limit = 20;

// --------------------------------------------------
// 最终启动
// --------------------------------------------------
motor0.init();
motor1.init();

Serial.println("Starting FOC Calibration...");
// 初始化 FOC (此时两个电机会依次发出 "滴" 声并微微转动寻找磁场零点)
motor0.initFOC();
motor1.initFOC();

command.add('T', doTarget, "target angle");

Serial.println("FOC Ready! Dual Motors Locked.");
Serial.println("Enter 'T3.14' to move motors 180 degrees.");
}

void loop() {
// 核心！FOC 底层高频循环，必须全速裸奔，不要加任何 delay()！
motor0.loopFOC();
motor1.loopFOC();

// 运动控制
motor0.move(target_angle);
motor1.move(target_angle);

// 接收串口指令
command.run();
}