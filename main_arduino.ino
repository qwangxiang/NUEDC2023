/*
串口命令参考：
    “Mode0/" --- 模式0
    "Mode1/" --- 模式1
    “<+xx+xx>” --- 设置速度和转角
*/


#include <SSD1306.h>
#include <Servo.h>
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#include <PS2X_lib.h>       //PS2手柄

//////////PS2引脚//////////////////
#define PS2_DAT        17  //14 A3
#define PS2_CMD        0  //15
#define PS2_SEL        16   //16
#define PS2_CLK        15  //17
////////OLED显示屏引脚///////////
#define OLED_DC 10
#define OLED_CLK 19
#define OLED_MOSI 13
#define OLED_RESET 12
/////////TB6612驱动引脚////
#define AIN1 11
#define AIN2 5
#define BIN1 6
#define BIN2 3
#define SERVO 9
/////////编码器引脚////////
#define ENCODER_L 8  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 4
#define ENCODER_R 7
#define DIRECTION_R 2
/////////按键引脚////////
#define KEY 18
#define T 0.156f
#define L 0.1445f
#define pi 3.1415926

// 串口相关的全局变量
String uart_receive_buf = "";   //串口接收用到的字符串
boolean newLineReceived = false; // 前一次数据结束标志
boolean startBit  = false;  //协议开始标志
int get_byte = 0;       // 接收到的 data byte

// 左右轮相关的全局变量
volatile long Velocity_L, Velocity_R ;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;   //左右轮速度
unsigned char servo;

// 速度控制相关全局变量
int Angle = 0, Velocity = 0;
float Velocity_KP = 0.3, Velocity_KI =  0.3;
float Target_A, Target_B;

// 运行模式
int state=0;
/*
0 ------ 停止模式
1 ------ 串口接收速度指令模式
*/



Servo myservo;  //创建一个舵机控制对象
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);


// 按键函数
unsigned char My_click (void) {
  static byte flag_key = 1; //按键按松开标志
  if (flag_key && (digitalRead(KEY) == 0)) {
    //如果发生单击事件
    flag_key = 0;
    if (digitalRead(KEY) == 0)  return 1; //M键
  }
  else if (digitalRead(KEY) == 1) flag_key = 1;
  return 0; //无按键按下
}

// 显示器相关函数
uint32_t oled_pow(uint8_t m, uint8_t n) 
{
    uint32_t result = 1;
    while (n--)result *= m;
    return result;
}

void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len) 
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)  {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        oled.drawchar(x + 6 * t, y, temp + '0');
    }
}

void oled_display()
{

    oled.drawstring(0, 0, "Mode:");
    if(state==0)
    {
        oled.drawstring(40, 0, "Mode0");
    }
    else if(state==1)
    {
        oled.drawstring(40, 0, "Mode1");
    }

    oled.drawstring(0, 3, "Angle:");
    if(Angle<0)
    {
        oled.drawstring(40, 3, "-");
    }
    else if(Angle==0)
    {
        oled.drawstring(40, 3, " ");
    }
    else
    {
        oled.drawstring(40, 3, "+");
    }
    OLED_ShowNumber(46, 3, abs(Angle), 2);

    oled.drawstring(0, 4, "Velocity:");
    if(Velocity<0)
    {
        oled.drawstring(64, 4, "-");
    }
    else if(Velocity==0)
    {
        oled.drawstring(64, 4, " ");
    }
    else
    {
        oled.drawstring(64, 4, "+");
    }
    OLED_ShowNumber(70, 4, abs(Velocity), 2);

    oled.display();
}

// 速度和电机控制
void Set_Pwm(int motora, int motorb) {
    if (motora > 0)      analogWrite(AIN2, motora), digitalWrite(AIN1, LOW); //赋值给PWM寄存器
    else                 digitalWrite(AIN1, HIGH), analogWrite(AIN2, 255 + motora); //赋值给PWM寄存器

    if (motorb > 0)       digitalWrite(BIN2, LOW), analogWrite(BIN1, motorb); //赋值给PWM寄存器
    else                  analogWrite(BIN1,255 +motorb), digitalWrite(BIN2, HIGH); //赋值给PWM寄存器
}

void Kinematic_Analysis(float velocity, float angle) {
    char K = 1;
    Target_A = velocity * (1 + T * tan(angle * pi / 180) / 2 / L);
    Target_B = velocity * (1 - T * tan(angle * pi / 180) / 2 / L); //后轮差速
    servo = 95 + angle * K;                      //舵机转向
    myservo.write(servo);                        // 指定舵机转向的角度
}

int Incremental_PI_A (int Target,int Encoder)
{   
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder;                                  //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
    if(Pwm>255)Pwm=255;                                 //限幅
    if(Pwm<-255)Pwm=-255;                                 //限幅  
    Last_bias=Bias;                                       //保存上一次偏差 
    return Pwm;                                           //增量输出
}
int Incremental_PI_B (int Target,int Encoder)
{   
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder;                                  //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
    if(Pwm>255)Pwm=255;                                 //限幅
    if(Pwm<-255)Pwm=-255;                                 //限幅  
    Last_bias=Bias;                                       //保存上一次偏差 
    return Pwm;                                           //增量输出
}



// 编码器读取函数
void READ_ENCODER_L() 
{
    if (digitalRead(ENCODER_L)==LOW) 
    {     //如果是下降沿触发的中断
        if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
        else      Velocity_L++;
    }
    else 
    {     //如果是上升沿触发的中断
        if (digitalRead(DIRECTION_L)==LOW)      Velocity_L++; //根据另外一相电平判定方向
        else     Velocity_L--;
    }
}

void READ_ENCODER_R() 
{
    if (digitalRead(ENCODER_R) == LOW) 
    { //如果是下降沿触发的中断
        if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++;//根据另外一相电平判定方向
        else      Velocity_R--;
    }
    else 
    {   //如果是上升沿触发的中断
        if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--; //根据另外一相电平判定方向
        else     Velocity_R++;
    }
} 




// 串口相关函数
void serialEvent()  
{
    while (Serial.available()) 
    {    
        get_byte = Serial.read();              //一个字节一个字节地读，下一句是读到的放入字符串数组中组成一个完成的数据包
        if( get_byte=='M'||get_byte=='<')
        {
        startBit= true;
        }
        if(startBit == true)
        {
        uart_receive_buf += (char) get_byte;     // 全双工串口可以不用在下面加延时，半双工则要加的//
        }  
        if (get_byte=='/'||get_byte=='>') 
        {
            newLineReceived = true; 
            startBit = false;
        }
    }
}

void Parse_str(String str)
{
    if(newLineReceived)
    {
        // Serial.println(str);
        if(str[0]=='M' && str[4]=='0')
        {   
            state  = 0;
        }
        else if(str[0]=='M' && str[4]=='1')
        {
            state  = 1;
        }
        else if(str.length()==8 && str[0]=='<' && state==1)
        {
            Angle = (str[2] - '0') * 10 + (str[3] - '0');
            if (str[1] == '-')
            {
                Angle -= 2*Angle ;
            }
    
            Velocity = (str[5] - '0') * 10 + (str[6] - '0');
            if (str[4] == '-')
            {
                Velocity -= 2*Velocity;
            }
        }
        else{}

        newLineReceived=false;
        uart_receive_buf="";
    }
}

void systick()
{
    int motora, motorb;

    sei();

    Kinematic_Analysis(Velocity, Angle);
    Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
    motora = Incremental_PI_A(Target_A, Velocity_Left); //===速度PI控制器
    motorb = Incremental_PI_B(Target_B, Velocity_Right); //===速度PI控制器
    Set_Pwm(motora, motorb);

    int click = My_click();
    if(click==1)
    {
        state=0;
    }
}


// 程序主体
void setup()
{
    char error;

    oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
    oled.clear();   // clears the screen and buffer

    pinMode(AIN1, OUTPUT);          //电机控制引脚
    pinMode(AIN2, OUTPUT);          //电机控制引脚，
    pinMode(BIN1, OUTPUT);          //电机速度控制引脚
    pinMode(BIN2, OUTPUT);          //电机速度控制引脚

    myservo.attach(SERVO);           // 选择控制的舵机

    pinMode(ENCODER_L, INPUT);       //编码器引脚
    pinMode(DIRECTION_L, INPUT);       //编码器引脚
    pinMode(ENCODER_R, INPUT);        //编码器引脚
    pinMode(DIRECTION_R, INPUT);       //编码器引脚

    delay(200);                      //延时等待初始化完成

    attachInterrupt(0, READ_ENCODER_R, CHANGE);           //开启外部中断 编码器接口1
    attachPinChangeInterrupt(4, READ_ENCODER_L, CHANGE);  //开启外部中断 编码器接口2
    MsTimer2::set(10,systick);       //使用Timer2设置5ms定时中断
    MsTimer2::start();               //中断使能
    Serial.begin(115200);            //开启串口                                                                             
}

void loop()
{
    Parse_str(uart_receive_buf);

    if(state==1)
    {
      
    }
    else
    {
        Angle = 0;
        Velocity = 0;
    }

    oled_display();
}
