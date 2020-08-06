/*
Headgear_Pro示例程序
 */

#include "Headgear_Pro.h"

//1. 关于端口宏的设置

//震动时间参数设置
#define SHORT_SHAKE_CYCLE 200 //短震动周期ms
#define SHAKE_TIME 2000 //震动时长
#define MOTOR_TIME 2000 //转头时长

//电机和震动电机端口
#define PORT_IN1 2
#define PORT_IN2 3
#define PORT_IN3 4
#define PORT_IN4 5
#define PORT_IN5 6
#define PORT_IN6 7
#define PORT_IN7 8
#define PORT_IN8 9
#define PORT_RIGHT_SHAKER 12
#define PORT_LEFT_SHAKER 13
//蓝牙端口(仅测试用)
#define BLUETOOTH_TX 10
#define BLUETOOTH_RX 11
#define BAUD_RATE 9600
//受力传感器宏
#define APORT_RIGHT_FRONT_PRESS 0
#define APORT_LEFT_FRONT_PRESS 1
#define APORT_RIGHT_BEHIND_PRESS 2
#define APORT_LEFT_BEHIND_PRESS 3

//2. 相关对象的初始化(实际使用中只需要初始化头套对象,但是这里为了调试以及控制增加了液晶屏以及蓝牙)

//蓝牙配对参数初始化(测试用)
int sliderVal, button, sliderId;
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth);

//头部装置初始化
HeadGear headGear(PORT_IN1,PORT_IN2,PORT_IN3,PORT_IN4,PORT_IN5,PORT_IN6,PORT_IN7,PORT_IN8,PORT_RIGHT_SHAKER,PORT_LEFT_SHAKER,APORT_RIGHT_FRONT_PRESS,APORT_LEFT_FRONT_PRESS,APORT_RIGHT_BEHIND_PRESS,APORT_LEFT_BEHIND_PRESS);

//液晶屏(测试用)
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI
void draw(float Fr,float Fl,float Br,float Bl,float roll)
{
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(0,15);
    u8g.print("Fr");
    u8g.setPrintPos(16,15);
    u8g.print(Fr);
    u8g.setPrintPos(48,15);
    u8g.print("Fl");
    u8g.setPrintPos(64,15);
    u8g.print(Fl);
    u8g.setPrintPos(0,32);
    u8g.print("Br");
    u8g.setPrintPos(16,32);
    u8g.print(Br);
    u8g.setPrintPos(48,32);
    u8g.print("Bl");
    u8g.setPrintPos(64,32);
    u8g.print(Bl);
    
    u8g.setPrintPos(0,64);
    u8g.print("roll");
    u8g.setPrintPos(32,64);
    u8g.print(roll);
    
}

//3. 初始化函数中需要进行的操作
void setup()
{
    //需要调动头套的初始化函数进行头套的初始化
    headGear.Setup();

    //蓝牙配对参数初始化(测试用)
    Serial.begin(BAUD_RATE);
    bluetooth.begin(BAUD_RATE);
    delay(100);
    Serial.println("setup complete");
}

//4. loop函数中需要进行的操作
void loop()
{
    //更新头套状态(必须操作)
    headGear.Update();
    
    /*
    下面都是调试内容,到时候可以做相应的替换
    */
    
    //调用人物的移动函数
    button = phone.getButton();
    if(button==0)//调用停止函数stop()
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.stop();
    }
    else if(button==1)//调用继续移动函数move()
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.move();
    }
    else if(button==2)//调用右转函数turnRight()
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.turnRight();
    }
    else if(button==3)//调用左转函数turnLeft()
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.turnLeft();
    }
    else if(button==4)//调用右侧震动函数来提示右侧障碍物
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.vibrate(1,0,0);//震动函数的使用说明:vibrate(<右侧是否震动>,<左侧是否震动>,<是否选择长震动模式>)
    }
    else if(button==5)//调用左侧震动函数来提示左侧障碍物
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.vibrate(0,1,0);
    }
    
    
    //屏幕显示(仅测试用)
    u8g.firstPage();
    do 
    {
        draw(headGear.getRightFrontForce(),headGear.getLeftFrontForce(),headGear.getRightBehindForce(),headGear.getLeftBehindForce(),headGear.getRoll());
    } while( u8g.nextPage() );
    
    /*
    //端口打印
    Serial.print("Yaw:");
    Serial.print(headGear.getYaw());
    Serial.print("\tPitch:");
    Serial.print(headGear.getPitch());
    Serial.print("\tRoll:");
    Serial.println(headGear.getRoll());
    Serial.print("Right Force: ");
    Serial.print(headGear.getRightForce());
    Serial.print("\tLeft Force: ");
    Serial.println(headGear.getLeftForce());
     */
    
}
