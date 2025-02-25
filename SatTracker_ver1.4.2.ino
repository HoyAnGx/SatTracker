#include <BluetoothSerial.h>

// 电机引脚定义
#define EN1_PIN   4   // X轴电机使能引脚
#define DIR1_PIN  16  // X轴电机DIR引脚
#define STEP1_PIN 17  // X轴电机STEP引脚 (方位角)

#define EN2_PIN   5  // Y轴电机使能引脚
#define DIR2_PIN  18  // Y轴电机DIR引脚
#define STEP2_PIN 19   // Y轴电机STEP引脚 (仰角)

// 蓝牙配置
BluetoothSerial SerialBT;

// LED 引脚定义
#define BLUE_LED_PIN 2 // 蓝灯引脚

// 电机参数
#define STEPS_PER_REV 200      // 电机每转步数
#define MICROSTEPPING 32       // 驱动器细分设置
const float STEPS_PER_DEGREE_AZ = (STEPS_PER_REV * MICROSTEPPING) / 360.0;  // 方位角每度步数
const float STEPS_PER_DEGREE_EL = (STEPS_PER_REV * MICROSTEPPING) / 90.0;   // 仰角每度步数

// 全局位置跟踪
long currentAzSteps = 0;
long currentElSteps = 0;

// 电机速度控制
const int PULSE_DELAY = 800; // 脉冲间隔（微秒）

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Gemini SatTracker");

  // 初始化电机引脚
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(EN1_PIN, OUTPUT);

  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(EN2_PIN, OUTPUT);

  // 初始化蓝灯引脚
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW); // 初始状态关闭

  // 禁用电机
  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, HIGH);

  // 归位流程
  homeMotors();
  Serial.println("系统初始化完成");
}

void loop() {
  handleBluetooth();
}

void homeMotors() {
  Serial.println("开始归位...");
  
  // X轴归位
  moveMotor(1, -STEPS_PER_REV * MICROSTEPPING); // 反向移动1圈
  currentAzSteps = 0; // 重置位置
  
  // Y轴归位
  moveMotor(2, -STEPS_PER_REV * MICROSTEPPING); 
  currentElSteps = 0;
  
  Serial.println("归位完成");
}

void handleBluetooth() {
  if (SerialBT.available()) {
    String data = SerialBT.readStringUntil('\n');
    data.trim(); // 去除首尾空格和换行符
    Serial.print("收到指令: ");
    Serial.println(data);

    // 闪烁蓝灯
    blinkBlueLED();

    // 解析数据格式 W20 A30
    int wIndex = data.indexOf('W'); // 查找 W 的位置
    int aIndex = data.indexOf('A'); // 查找 A 的位置

    // 确保 W 和 A 都存在，并且 W 在 A 之前
    if (wIndex != -1 && aIndex != -1 && wIndex < aIndex) {
      // 提取方位角（AZ）
      String azStr = data.substring(wIndex + 1, aIndex);
      azStr.trim(); // 去除空格
      
      // 提取仰角（EL）
      String elStr = data.substring(aIndex + 1);
      elStr.trim(); // 去除空格

      // 如果 elStr 中包含空格或其他字符，只取第一个有效部分
      int spaceIndex = elStr.indexOf(' ');
      if (spaceIndex != -1) {
        elStr = elStr.substring(0, spaceIndex); // 只取第一个有效部分
      }

      // 校验数据是否为有效数字
      if (isValidNumber(azStr) && isValidNumber(elStr)) {
        float az = azStr.toFloat();
        float el = elStr.toFloat();
        
        // 将负数转换为正数
        if (az < 0) az = -az;
        if (el < 0) el = -el;
        
        // 限制角度范围
        az = fmod(az, 360.0);   // 方位角0-360
        el = constrain(el, 0, 90.0); // 仰角0-90
        
        // 计算目标步数
        long targetAzSteps = round(az * STEPS_PER_DEGREE_AZ);
        long targetElSteps = round(el * STEPS_PER_DEGREE_EL);
        
        // 计算移动差值（考虑360°到0°的平滑过渡）
        long moveAz = calculateAzSteps(currentAzSteps, targetAzSteps);
        long moveEl = targetElSteps - currentElSteps;
        
        // 执行移动
        if (moveAz != 0) {
          moveMotor(1, moveAz);
          currentAzSteps = targetAzSteps;
        }
        if (moveEl != 0) {
          moveMotor(2, moveEl);
          currentElSteps = targetElSteps;
        }
      } else {
        Serial.println("错误: 无效的角度数据");
      }
    } else {
      Serial.println("错误: 数据格式不正确");
    }
  }
}

// 计算方位角步数（考虑360°到0°的平滑过渡）
long calculateAzSteps(long currentSteps, long targetSteps) {
  long delta = targetSteps - currentSteps;
  
  // 如果差值超过半圈，选择更短的方向
  if (delta > STEPS_PER_REV * MICROSTEPPING / 2) {
    delta -= STEPS_PER_REV * MICROSTEPPING; // 反转方向
  } else if (delta < -STEPS_PER_REV * MICROSTEPPING / 2) {
    delta += STEPS_PER_REV * MICROSTEPPING; // 反转方向
  }
  
  return delta;
}

void moveMotor(int axis, long steps) {
  int stepPin, dirPin, enPin;
  
  // 选择电机
  if (axis == 1) { // X轴
    stepPin = STEP1_PIN;
    dirPin = DIR1_PIN;
    enPin = EN1_PIN;
  } else { // Y轴
    stepPin = STEP2_PIN;
    dirPin = DIR2_PIN;
    enPin = EN2_PIN;
  }

  // 使能电机
  digitalWrite(enPin, LOW);
  
  // 设置方向
  digitalWrite(dirPin, (steps > 0) ? HIGH : LOW);
  steps = abs(steps);

  // 生成脉冲
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_DELAY);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PULSE_DELAY);
  }

  // 禁用电机
  digitalWrite(enPin, HIGH);
  
  Serial.print("电机");
  Serial.print(axis);
  Serial.print(" 移动完成，步数: ");
  Serial.println(steps);
}

// 校验字符串是否为有效数字
bool isValidNumber(String str) {
  for (char c : str) {
    if (!isdigit(c) && c != '.' && c != '-') {
      return false;
    }
  }
  return true;
}

// 闪烁蓝灯
void blinkBlueLED() {
  for (int i = 0; i < 3; i++) { // 闪烁 3 次
    digitalWrite(BLUE_LED_PIN, HIGH); // 点亮
    delay(100); // 持续 100ms
    digitalWrite(BLUE_LED_PIN, LOW);  // 熄灭
    delay(100); // 间隔 100ms
  }
}
