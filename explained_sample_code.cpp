#include <SSD1306.h>      // 导入SSD1306库，用于控制OLED屏幕
#include <Servo.h>        // 导入舵机库，用于控制舵机
#include <PinChangeInt.h> // 导入PinChangeInt库，用于处理引脚中断
#include <MsTimer2.h>     // 导入MsTimer2库，用于处理定时中断
#include <PS2X_lib.h>     // 导入PS2X库，用于处理PS2手柄的输入

Servo myservo; // 创建一个舵机控制对象
PS2X ps2x;     // 创建一个用于处理PS2手柄输入的对象

// PS2手柄的引脚定义
#define PS2_DAT 17 // PS2手柄的数据线连接的引脚
#define PS2_CMD 0  // PS2手柄的命令线连接的引脚
#define PS2_SEL 16 // PS2手柄的选择线连接的引脚
#define PS2_CLK 15 // PS2手柄的时钟线连接的引脚

// OLED显示屏的引脚定义
#define OLED_DC 10    // OLED屏幕的数据/命令选择线连接的引脚
#define OLED_CLK 19   // OLED屏幕的时钟线连接的引脚
#define OLED_MOSI 13  // OLED屏幕的主机数据输出线连接的引脚
#define OLED_RESET 12 // OLED屏幕的复位线连接的引脚

// TB6612驱动的引脚定义
#define AIN1 11 // TB6612驱动的A输入1连接的引脚
#define AIN2 5  // TB6612驱动的A输入2连接的引脚
#define BIN1 6  // TB6612驱动的B输入1连接的引脚
#define BIN2 3  // TB6612驱动的B输入2连接的引脚
#define SERVO 9 // 用于控制舵机的PWM输出引脚

// 编码器的引脚定义
#define ENCODER_L 8   // 左侧编码器的信号输入引脚
#define DIRECTION_L 4 // 左侧编码器的方向输入引脚
#define ENCODER_R 7   // 右侧编码器的信号输入引脚
#define DIRECTION_R 2 // 右侧编码器的方向输入引脚

// 按键的引脚定义
#define KEY 18 // 连接按键的输入引脚

#define T 0.156f
#define L 0.1445f
#define pi 3.1415926

String uart1_receive_buf = "";   // 定义一个用于接收串口数据的字符串
int get_byte = 0;                // 接收到的数据字节
boolean newLineReceived = false; // 新行接收标志
boolean startBit = false;        // 协议开始标志

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0); // 创建一个OLED显示屏对象

// 巡线相关的变量
int Sensor_Left, Sensor_Middle, Sensor_Right, Sensor;

// 左右轮编码器数据
volatile long Velocity_L, Velocity_R;

// 左右轮速度
int Velocity_Left, Velocity_Right = 0, Velocity, Angle;
char Flag_Direction, Flag_Way = 1;

// 速度PID控制器的参数
float Velocity_KP = 0.3, Velocity_KI = 0.3;

// 停止标志位和上位机相关变量
unsigned char Flag_Stop = 0, PID_Send, Flash_Send, Bluetooth_Velocity = 25;
float Target_A, Target_B;

// 电池电压采样变量
int Battery_Voltage, ps2_data;

unsigned char servo, PS2_LY, PS2_RX, CCD_Yuzhi, CCD_Zhongzhi;
unsigned char ADV[128] = {0};

void (*resetFunc)(void) = 0; // Reset func，软复位函数

// 函数功能：计算线性CCD的中值
void Find_CCD_Zhongzhi(void)
{
  // 静态变量
  static unsigned int i, j, Left, Right, Last_CCD_Zhongzhi, value1_max, value1_min;

  // 动态阈值算法，初始化最大值
  value1_max = ADV[0];

  // 从第6个点开始，两边各去掉5个点，找出最大值
  for (i = 5; i < 123; i++)
  {
    if (value1_max <= ADV[i])
      value1_max = ADV[i];
  }

  // 初始化最小值
  value1_min = ADV[0];

  // 从第6个点开始，两边各去掉5个点，找出最小值
  for (i = 5; i < 123; i++)
  {
    if (value1_min >= ADV[i])
      value1_min = ADV[i];
  }

  // 计算出本次中线提取的阈值
  CCD_Yuzhi = (value1_max + value1_min) / 2;

  // 寻找左边跳变沿
  for (i = 5; i < 118; i++)
  {
    if (ADV[i] > CCD_Yuzhi && ADV[i + 1] > CCD_Yuzhi && ADV[i + 2] > CCD_Yuzhi && ADV[i + 3] < CCD_Yuzhi && ADV[i + 4] < CCD_Yuzhi && ADV[i + 5] < CCD_Yuzhi)
    {
      Left = i;
      break;
    }
  }

  // 寻找右边跳变沿
  for (j = 118; j > 5; j--)
  {
    if (ADV[j] < CCD_Yuzhi && ADV[j + 1] < CCD_Yuzhi && ADV[j + 2] < CCD_Yuzhi && ADV[j + 3] > CCD_Yuzhi && ADV[j + 4] > CCD_Yuzhi && ADV[j + 5] > CCD_Yuzhi)
    {
      Right = j;
      break;
    }
  }

  // 计算中线位置
  CCD_Zhongzhi = (Right + Left) / 2;

  // 以下代码被注释掉，原本用于计算中线的偏差，如果偏差太大则取上一次的值，并保存上一次的偏差
  // if((CCD_Zhongzhi-Last_CCD_Zhongzhi)>90||(CCD_Zhongzhi-Last_CCD_Zhongzhi)<-90)
  //  CCD_Zhongzhi=Last_CCD_Zhongzhi;
  // Last_CCD_Zhongzhi=CCD_Zhongzhi;
}

// 函数功能：微秒级延时
void Dly_us(void)
{
  for (char ii = 0; ii < 10; ii++)
    ;
}

// 函数功能：读取线性CCD数据
void RD_TSL(void)
{
  unsigned char i = 0, tslp = 0;

  // 设置16号引脚为高电平，17号引脚为低电平，然后延时
  digitalWrite(16, HIGH);
  digitalWrite(17, LOW);
  Dly_us();

  // 设置17号引脚为高电平，16号引脚为低电平，然后延时
  digitalWrite(17, HIGH);
  digitalWrite(16, LOW);
  Dly_us();

  // 设置16号引脚为高电平，17号引脚为低电平，然后延时
  digitalWrite(16, HIGH);
  digitalWrite(17, LOW);
  Dly_us();

  // 循环128次，读取CCD数据
  for (i = 0; i < 128; i++)
  {
    // 设置16号引脚为低电平
    digitalWrite(16, LOW);

    // 延时，调节曝光时间
    Dly_us();
    Dly_us();
    // 读取1号模拟引脚的值，右移2位（即除以4），然后保存至ADV数组
    ADV[tslp++] = analogRead(1) >> 2;

    // 设置16号引脚为高电平，然后延时
    digitalWrite(16, HIGH);
    Dly_us();
  }
}
// 这个函数的功能是通过模拟输入读取CCD数据，然后存储在数组中。

// 函数功能：选择运行的模式
unsigned char select(void)
{
  // `count` 用来计数，`AngleX` 是一个阈值，`flag` 是一个标志位，初始化为1
  int count, AngleX = 130;
  static unsigned char flag = 1;

  // 调用 `oled_show_once()` 函数进行 OLED 显示
  oled_show_once();

  // 计算 `Velocity_R` 的绝对值并赋值给 `count`
  count = abs(Velocity_R);

  // 根据 `count` 的值选择运行的模式
  if (count <= AngleX)
    Flag_Way = 0; // APP遥控模式
  else if (count > AngleX && count <= 2 * AngleX)
    Flag_Way = 1; // PS2遥控模式
  else if (count > 2 * AngleX && count <= 3 * AngleX)
    Flag_Way = 2; // CCD巡线模式
  else if (count > 3 * AngleX && count <= 4 * AngleX)
    Flag_Way = 3; // 电磁巡线模式
  else
    Velocity_R = 0; // 如果 `count` 的值超过 `4*AngleX`，则设置 `Velocity_R` 为0

  // 如果 KEY 引脚的电平为0，清除 OLED 显示并将 `flag` 设置为0
  if (digitalRead(KEY) == 0)
    oled.clear(), flag = 0;

  // 返回 `flag` 的值
  return flag;
}

// 函数功能：按键扫描
unsigned char My_click(void)
{
  // `flag_key` 是按键按下的标志位，初始化为1
  static byte flag_key = 1;

  // 如果 `flag_key` 为1且 KEY 引脚的电平为0，表示发生了单击事件
  if (flag_key && (digitalRead(KEY) == 0))
  {
    flag_key = 0;
    if (digitalRead(KEY) == 0)
      return 1; // 如果 KEY 引脚的电平仍为0，返回1，表示 M键被按下
  }
  else if (digitalRead(KEY) == 1)
    flag_key = 1; // 如果 KEY 引脚的电平为1，将 `flag_key` 设置为1

  // 如果没有按键被按下，返回0
  return 0;
}

// 函数功能：求次方的函数
uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;
  // 使用循环计算 m 的 n 次方
  while (n--)
    result *= m;
  return result;
}

// 函数功能：显示变量
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
  uint8_t t, temp;
  uint8_t enshow = 0;
  for (t = 0; t < len; t++)
  {
    // 取 num 的第 t+1 位数字
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    // 在 OLED 显示屏上显示该数字
    oled.drawchar(x + 6 * t, y, temp + '0');
  }
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
**************************************************************************/
void Set_Pwm(int motora, int motorb)
{
  // 控制电机 A 的正反转和速度
  if (motora > 0)
    analogWrite(AIN2, motora), digitalWrite(AIN1, LOW); // 电机A正转，速度为motora
  else
    digitalWrite(AIN1, HIGH), analogWrite(AIN2, 255 + motora); // 电机A反转，速度为-motora

  // 控制电机 B 的正反转和速度
  if (motorb > 0)
    digitalWrite(BIN2, LOW), analogWrite(BIN1, motorb); // 电机B正转，速度为motorb
  else
    analogWrite(BIN1, 255 + motorb), digitalWrite(BIN2, HIGH); // 电机B反转，速度为-motorb
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
/**************************************************************************/
unsigned char Turn_Off()
{
  byte temp;
  // 如果停止标志 Flag_Stop 为1，或者电池电压低于700（可能是700毫伏），则关闭电机
  if (Flag_Stop == 1 || Battery_Voltage < 700)
  {
    temp = 1;
    digitalWrite(AIN1, LOW); // 停止驱动电机A
    digitalWrite(AIN2, LOW); // 停止驱动电机A
    digitalWrite(BIN1, LOW); // 停止驱动电机B
    digitalWrite(BIN2, LOW); // 停止驱动电机B
  }
  else
    temp = 0;
  return temp;
}

/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
//**************************************************************************/
void Kinematic_Analysis(float velocity, float angle)
{
  char K = 1;
  // 计算目标速度 Target_A 和 Target_B，这里的公式来源于小车运动的数学模型
  Target_A = velocity * (1 + T * tan(angle * pi / 180) / 2 / L);
  Target_B = velocity * (1 - T * tan(angle * pi / 180) / 2 / L); // 后轮差速
  servo = 95 + angle * K;                                        // 根据转角计算舵机的目标位置
  myservo.write(servo);                                          // 设置舵机的位置
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

// 增量PI控制器，用于电机A的速度控制
int Incremental_PI_A(int Encoder, int Target)
{
  static float Bias, Pwm, Last_bias;
  Bias = Encoder - Target; // 计算偏差，偏差=实际速度-目标速度
  // 根据增量式PI控制器的公式计算PWM值，注意这里只使用了PI控制，没有使用D控制
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  if (Pwm > 255)
    Pwm = 255; // 限制PWM的最大值为255
  if (Pwm < -255)
    Pwm = -255;     // 限制PWM的最小值为-255
  Last_bias = Bias; // 保存上一次的偏差
  return Pwm;       // 返回计算得到的PWM值
}

void control()
{                                                     // 主控制函数
  int Temp, Temp2, Motora, Motorb;                    // 临时变量，Motora和Motorb用于存储两个电机的控制信号
  static float Voltage_All;                           // 用于记录电池电压采样的总和
  static unsigned char Position_Count, Voltage_Count; // 用于记录电池电压和位置控制的采样次数
  sei();                                              // 全局中断开启，这是在AVR单片机中常用的使能全局中断的函数
  Velocity_Left = Velocity_L;
  Velocity_L = 0; // 读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_Right = Velocity_R;
  Velocity_R = 0;                                      // 读取右轮编码器数据，并清零
  Get_RC();                                            // 读取遥控器的输入
  Kinematic_Analysis(Velocity, Angle);                 // 根据遥控器的输入进行运动学分析
  Motora = Incremental_PI_A(Target_A, Velocity_Left);  // 左轮速度PI控制器，返回的赋值给电机A
  Motorb = Incremental_PI_B(Target_B, Velocity_Right); // 右轮速度PI控制器，返回的赋值给电机B
  if (Turn_Off() == 0)
    Set_Pwm(Motora, Motorb); // 如果不存在异常，使能电机的PWM输出
  Temp2 = analogRead(0);     // 采集一下电池电压
  Voltage_Count++;           // 采样次数加一
  Voltage_All += Temp2;      // 将当前采样值加到总和中
  if (Voltage_Count == 200)
    Battery_Voltage = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0; // 每200次采样，计算平均电压，并清零总和和计数器
  Temp = My_click();                                                                 // 检查按键输入
  if (Temp == 1)
    Flag_Stop = !Flag_Stop; // 如果按键被按下，改变停止标志的状态
}

void Get_RC(void)
{                         // 获取遥控器输入的函数
  int sum;                // 临时变量，用于计算电磁传感器的偏差
  char Yuzhi = 2;         // 设定一个阈值，用于摇杆操作的死区处理
  static float Last_Bias; // 保存上一次的偏差
  float Bias, LY, RX;     // Bias用于保存当前偏差，LY和RX分别表示遥控器的两个轴的输入
  if (Flag_Way == 0)
  { // 蓝牙控制
    // 根据Flag_Direction的值设置机器人的运动方向和速度
  }
  else if (Flag_Way == 1)
  { // PS2控制
    // 根据PS2遥控器的摇杆输入设置机器人的速度和方向
  }
  else if (Flag_Way == 2)
  { // CCD巡线
    // 根据CCD传感器的输入，进行PD控制，设置机器人的速度和方向
  }
  else if (Flag_Way == 3)
  { // 电磁巡线
    // 根据电磁传感器的输入，进行PD控制，设置机器人的速度和方向
  }
  if (Angle < -45)
    Angle = -45; //////舵机角度限制
  if (Angle > 45)
    Angle = 45; // 舵机角度限制
}

/***************函数功能：遥控**********/
void Get_RC(void)
{
  int sum;                // 用于计算电磁传感器的和
  char Yuzhi = 2;         // 初始化摇杆死区阈值
  static float Last_Bias; // 用于存储上一次的偏差值
  float Bias, LY, RX;     // Bias: 偏差值, LY: PS2左摇杆Y值, RX: PS2右摇杆X值

  // 蓝牙控制，如果Flag_Way等于0
  if (Flag_Way == 0)
  {
    // 根据Flag_Direction设置速度和角度
    if (Flag_Direction == 0)
      Velocity = 0, Angle = 0; // 停止
    else if (Flag_Direction == 1)
      Velocity = Bluetooth_Velocity, Angle = 0; // 前进
    else if (Flag_Direction == 4)
      Velocity = Bluetooth_Velocity, Angle = 36; // 右前
    else if (Flag_Direction == 2)
      Velocity = -Bluetooth_Velocity, Angle = 0; // 后退
    else if (Flag_Direction == 3)
      Velocity = Bluetooth_Velocity, Angle = -36; // 左前
  }
  // PS2控制，如果Flag_Way等于1
  else if (Flag_Way == 1)
  {
    LY = PS2_LY - 128; // 计算偏差
    RX = PS2_RX - 128;
    // 设置摇杆死区，防止抖动异常
    if (LY > -Yuzhi && LY < Yuzhi)
      LY = 0;
    if (RX > -Yuzhi && RX < Yuzhi)
      RX = 0;
    Velocity = -LY / 5; // 速度与摇杆力度相关
    Angle = RX / 4;
  }
  // CCD巡线，如果Flag_Way等于2
  else if (Flag_Way == 2)
  {
    RD_TSL();                                    // 读取CCD数据
    Find_CCD_Zhongzhi();                         // 计算CCD中值
    Velocity = 28;                               // CCD巡线模式的速度
    Bias = CCD_Zhongzhi - 64;                    // 提取偏差
    Angle = Bias * 0.6 + (Bias - Last_Bias) * 3; // PD控制
    Last_Bias = Bias;                            // 保存上一次的偏差
  }
  // 电磁巡线，如果Flag_Way等于3
  else if (Flag_Way == 3)
  {
    Sensor_Left = analogRead(1);   // 读取左侧传感器数据
    Sensor_Middle = analogRead(2); // 读取中间传感器数据
    Sensor_Right = analogRead(3);  // 读取右侧传感器数据
    // 计算三个传感器的和
    if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
    {
      sum = Sensor_Left * 1 + Sensor_Middle * 50 + Sensor_Right * 99; // 归一化处理
      Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right);    // 求偏差
    }
    Velocity = 18;      // 电磁巡线模式下的速度
    Bias = Sensor - 50; // 提取偏差
    Angle = abs(Bias) * Bias * 0.02 + Bias * 0.074 + (Bias - Last_Bias) * 1;
    Last_Bias = Bias; // 保存上一次的偏差
  }
  // 舵机角度限制
  if (Angle < -45)
    Angle = -45;
  if (Angle > 45)
    Angle = 45;
}

void OLED()
{
  char i, t;
  // 如果Flag_Way等于0，显示SPEED和RX，以及与蓝牙速度和方向标志相关的信息
  if (Flag_Way == 0)
  {
    oled.drawstring(00, 00, "SPEED");
    OLED_ShowNumber(45, 00, Bluetooth_Velocity, 3); // 显示蓝牙速度
    oled.drawstring(00, 01, "RX");
    OLED_ShowNumber(30, 01, Flag_Direction, 3); // 显示方向标志数据
  }
  // 如果Flag_Way等于1，显示LY和RX，以及与PS2控制器的LY和RX相关的信息
  else if (Flag_Way == 1)
  {
    oled.drawstring(00, 0, "LY");
    OLED_ShowNumber(15, 0, PS2_LY, 3); // 显示PS2控制器的LY数据
    oled.drawstring(40, 0, "RX");
    OLED_ShowNumber(55, 0, PS2_RX, 3); // 显示PS2控制器的RX数据
  }
  // 如果Flag_Way等于2，调用OLED_Show_CCD函数显示CCD相关信息，以及Z和Y的值
  else if (Flag_Way == 2)
  {
    OLED_Show_CCD();
    oled.drawstring(00, 01, "Z");
    OLED_ShowNumber(35, 01, CCD_Zhongzhi, 3);
    oled.drawstring(70, 01, "Y");
    OLED_ShowNumber(95, 01, CCD_Yuzhi, 3);
  }
  // 如果Flag_Way等于3，显示传感器数据
  else if (Flag_Way == 3)
  {
    oled.drawstring(00, 0, "L");
    OLED_ShowNumber(10, 0, Sensor_Left, 4);
    oled.drawstring(40, 0, "M");
    OLED_ShowNumber(50, 0, Sensor_Middle, 4);
    oled.drawstring(80, 0, "R");
    OLED_ShowNumber(90, 0, Sensor_Right, 4);
    oled.drawstring(0, 01, "Z");
    OLED_ShowNumber(20, 01, Sensor, 3);
  }
  // 显示编码器数据
  oled.drawstring(00, 02, "EncoLEFT");
  if (Velocity_Left < 0)
    oled.drawstring(80, 02, "-"),
        OLED_ShowNumber(95, 02, -Velocity_Left, 3); // 显示左侧速度
  else
    oled.drawstring(80, 02, "+"),
        OLED_ShowNumber(95, 02, Velocity_Left, 3); // 显示左侧速度
  oled.drawstring(00, 03, "EncoRIGHT");
  if (Velocity_Right < 0)
    oled.drawstring(80, 03, "-"),
        OLED_ShowNumber(95, 03, -Velocity_Right, 3); // 显示右侧速度
  else
    oled.drawstring(80, 03, "+"),
        OLED_ShowNumber(95, 03, Velocity_Right, 3); // 显示右侧速度
  // 显示电池电压
  oled.drawstring(00, 4, "VOLTAGE:");
  oled.drawstring(71, 4, ".");
  oled.drawstring(93, 4, "V");
  OLED_ShowNumber(58, 4, Battery_Voltage / 100, 2);
  OLED_ShowNumber(81, 4, Battery_Voltage % 100, 2);
  // 显示停止标志
  if (Flag_Stop == 0)
    oled.drawstring(103, 04, "O-N");
  if (Flag_Stop == 1)
    oled.drawstring(103, 04, "OFF");
  // 显示模式和伺服状态
  oled.drawstring(00, 05, "MODE-");
  if (Flag_Way == 0)
    oled.drawstring(40, 05, "APP");
  else if (Flag_Way == 1)
    oled.drawstring(40, 05, "PS2");
  else if (Flag_Way == 2)
    oled.drawstring(40, 05, "CCD");
  else if (Flag_Way == 3)
    oled.drawstring(40, 05, "ELE");

  oled.drawstring(80, 05, "S");
  OLED_ShowNumber(95, 05, servo, 3);
  oled.display();
}

// 开机显示一次的内容
void oled_show_once(void)
{
  oled.drawstring(0, 0, "Turn Right Wheel");
  oled.drawstring(0, 1, "TO Select Mode");
  oled.drawstring(0, 2, "Current Mode Is");
  if (Flag_Way == 0)
    oled.drawstring(50, 3, "APP");
  if (Flag_Way == 1)
    oled.drawstring(50, 3, "PS2");
  if (Flag_Way == 2)
    oled.drawstring(50, 3, "CCD");
  if (Flag_Way == 3)
    oled.drawstring(50, 3, "ELE");
  oled.drawstring(0, 4, "Press User Key");
  oled.drawstring(0, 5, "TO End Selection");
  OLED_ShowNumber(0, 6, abs(Velocity_R), 4);
  oled.display();
}

/***********函数功能：初始化 相当于STM32里面的Main函数 ************/
void setup()
{
  char error; // 错误变量，用于存储可能出现的错误

  oled.ssd1306_init(SSD1306_SWITCHCAPVCC); // 初始化OLED显示屏
  oled.clear();                            // 清除显示屏和缓冲区

  pinMode(AIN1, OUTPUT); // 设置AIN1为输出模式，用于电机控制
  pinMode(AIN2, OUTPUT); // 设置AIN2为输出模式，用于电机控制
  pinMode(BIN1, OUTPUT); // 设置BIN1为输出模式，用于电机速度控制
  pinMode(BIN2, OUTPUT); // 设置BIN2为输出模式，用于电机速度控制
  myservo.attach(SERVO); // 选择控制的舵机

  pinMode(ENCODER_L, INPUT);   // 设置ENCODER_L为输入模式，用于从编码器接收数据
  pinMode(DIRECTION_L, INPUT); // 设置DIRECTION_L为输入模式，用于从编码器接收数据
  pinMode(ENCODER_R, INPUT);   // 设置ENCODER_R为输入模式，用于从编码器接收数据
  pinMode(DIRECTION_R, INPUT); // 设置DIRECTION_R为输入模式，用于从编码器接收数据
  pinMode(KEY, INPUT);         // 设置KEY为输入模式，用于按键输入
  delay(200);                  // 延时200ms等待初始化完成

  attachInterrupt(0, READ_ENCODER_R, CHANGE);          // 在第0号中断引脚上，当状态改变时，调用函数READ_ENCODER_R
  attachPinChangeInterrupt(4, READ_ENCODER_L, CHANGE); // 在第4号中断引脚上，当状态改变时，调用函数READ_ENCODER_L

  while (select())
  {
  } // 持续调用select()函数，直到它返回false，选择运行模式

  MsTimer2::set(10, control); // 使用Timer2设置10ms的定时中断，并在中断时调用control函数
  MsTimer2::start();          // 启动定时器中断

  // Flag_Way变量决定了控制方式
  if (Flag_Way == 0)
    Serial.begin(115200); // 如果Flag_Way等于0，通过蓝牙进行控制，开启波特率为115200的串口
  else if (Flag_Way == 1)
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false); // 如果Flag_Way等于1，通过PS2手柄进行控制，配置PS2手柄
  else if (Flag_Way == 2)
    pinMode(16, OUTPUT), pinMode(17, OUTPUT); // 如果Flag_Way等于2，通过CCD相机进行控制，设置16和17号引脚为输出模式，这可能与CCD相机的某些控制线连接
}

/* 主循环 */
void loop()
{
  static char flag; // 静态变量，用于标识一些状态，在函数调用结束后不会销毁,此处未使用
  int Voltage_Temp; // 临时电压变量,但在这段代码中并未使用
  OLED();           // 调用OLED显示函数，具体实现未给出
  if (Flag_Way == 0)
  {                               // 如果Flag_Way为0，使用串口接收的方式控制小车
    Parse_str(uart1_receive_buf); // 解析通过串口接收到的字符串，并执行相应的操作
  }
  else if (Flag_Way == 1)
  {                               // 如果Flag_Way为1，使用PS2控制器控制小车
    ps2x.read_gamepad(false, 0);  // 读取PS2控制器的状态
    PS2_LY = ps2x.Analog(PSS_LY); // 获取PS2控制器左摇杆的Y轴数据
    PS2_RX = ps2x.Analog(PSS_RX); // 获取PS2控制器右摇杆的X轴数据
  }
}

/* 读取左边编码器的函数 */
void READ_ENCODER_L()
{
  if (digitalRead(ENCODER_L) == LOW)
  { // 如果编码器的信号是低电平
    if (digitalRead(DIRECTION_L) == LOW)
      Velocity_L--; // 如果方向是低电平，那么速度减一
    else
      Velocity_L++; // 否则速度加一
  }
  else
  { // 如果编码器的信号是高电平
    if (digitalRead(DIRECTION_L) == LOW)
      Velocity_L++; // 如果方向是低电平，那么速度加一
    else
      Velocity_L--; // 否则速度减一
  }
}

/* 读取右边编码器的函数 */
void READ_ENCODER_R()
{
  if (digitalRead(ENCODER_R) == LOW)
  { // 如果编码器的信号是低电平
    if (digitalRead(DIRECTION_R) == LOW)
      Velocity_R++; // 如果方向是低电平，那么速度加一
    else
      Velocity_R--; // 否则速度减一
  }
  else
  { // 如果编码器的信号是高电平
    if (digitalRead(DIRECTION_R) == LOW)
      Velocity_R--; // 如果方向是低电平，那么速度减一
    else
      Velocity_R++; // 否则速度加一
  }
}

/* 串口接收中断函数 */
void serialEvent()
{
  while (Serial.available())
  {                           // 当有串口数据可读时
    get_byte = Serial.read(); // 读取一个字节的数据
    if (get_byte == 'B' || get_byte == '<' || get_byte == '#')
    {
      startBit = true; // 如果读取到的是'B'，'<'或'#'，那么设置开始位为true
    }
    if (startBit == true)
    {
      uart1_receive_buf += (char)get_byte; // 如果开始位为true，则将读取到的数据添加到接收缓冲区
    }
    if (get_byte == '\n' || get_byte == '>')
    {
      newLineReceived = true; // 如果读取到的是换行符或者'>'，那么设置新行接收标志为true
      startBit = false;       // 并且设置开始位为false
    }
  }
}

// 定义函数Parse_str，输入参数为字符串str
void Parse_str(String str)
{
  // 定义变量index, len, i并初始化为0
  int index = 0, len = 0, i = 0;

  // 如果接收到新的一行数据
  if (newLineReceived)
  {
    // 如果字符串以 "BUPD" 或 "<BUPD>" 开头，设置 Flag_Direction 为 1
    if ((str[0] == 'B' && (str[1] == 'U') && (str[2] == 'P') && (str[3] == 'D')) || (str[0] == '<' && (str[1] == 'B') && (str[2] == 'U') && (str[3] == 'P') && (str[4] == 'D') && (str[5] == '>')))
    {
      Flag_Direction = 1;
    }
    // 如果字符串以 "BDND" 或 "<BDND>" 开头，设置 Flag_Direction 为 2
    else if ((str[0] == 'B' && (str[1] == 'D') && (str[2] == 'N') && (str[3] == 'D')) || (str[0] == '<' && (str[1] == 'B') && (str[2] == 'D') && (str[3] == 'N') && (str[4] == 'D') && (str[5] == '>')))
    {
      Flag_Direction = 2;
    }
    // 如果字符串以 "BLTD" 或 "<BLTD>" 开头，设置 Flag_Direction 为 3
    else if ((str[0] == 'B' && (str[1] == 'L') && (str[2] == 'T') && (str[3] == 'D')) || (str[0] == '<' && (str[1] == 'B') && (str[2] == 'L') && (str[3] == 'T') && (str[4] == 'D') && (str[5] == '>')))
    {
      Flag_Direction = 3;
    }
    // 如果字符串以 "BRTD" 或 "<BRTD>" 开头，设置 Flag_Direction 为 4
    else if ((str[0] == 'B' && (str[1] == 'R') && (str[2] == 'T') && (str[3] == 'D')) || (str[0] == '<' && (str[1] == 'B') && (str[2] == 'R') && (str[3] == 'T') && (str[4] == 'D') && (str[5] == '>')))
    {
      Flag_Direction = 4;
    }
    // 如果字符串以 "BSTD" 或 "<BU>" 开头，设置 Flag_Direction 为 0
    else if ((str[0] == 'B' && (str[1] == 'S') && (str[2] == 'T') && (str[3] == 'D')) || (str[0] == '<' && (str[1] == 'B') && (str[4] == 'U') && (str[5] == '>')))
    {
      Flag_Direction = 0;
    }
    // 如果字符串以 "BUAD" 开头，减小 Bluetooth_Velocity 的值
    else if (str[0] == 'B' && (str[1] == 'U') && (str[2] == 'A') && (str[3] == 'D'))
    {
      Bluetooth_Velocity = Bluetooth_Velocity - 5;
      if (Bluetooth_Velocity <= 10)
        Bluetooth_Velocity = 10;
    }
    // 如果字符串以 "BUMD" 开头，增加 Bluetooth_Velocity 的值
    else if (str[0] == 'B' && (str[1] == 'U') && (str[2] == 'M') && (str[3] == 'D'))
    {
      Bluetooth_Velocity = Bluetooth_Velocity + 10;
      if (Bluetooth_Velocity >= 25)
        Bluetooth_Velocity = 25;
    }
    // 如果字符串以 "BUKD" 开头，执行开灯
    else if (str[0] == 'B' && (str[1] == 'U') && (str[2] == 'K') // 开灯
             && (str[3] == 'D'))
    {
    }
    // 如果字符串以 "BUSD" 开头，执行开灯
    else if (str[0] == 'B' && (str[1] == 'U') && (str[2] == 'S') // 鸣笛
             && (str[3] == 'D'))
    {
    }
    newLineReceived = false;
    uart1_receive_buf = "";
  }
}