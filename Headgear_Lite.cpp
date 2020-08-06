/*
Headgear_Lite示例程序
 */

#include "Headgear_Lite.h"

//1. 关于端口宏的设置

//电机和震动电机端口
#define PORT_ENA 3
#define PORT_IN1 4
#define PORT_IN2 5
#define PORT_IN3 6
#define PORT_IN4 7
#define PORT_RIGHT_SHAKER 12
#define PORT_LEFT_SHAKER 13
//蓝牙端口(仅测试用)
#define BLUETOOTH_TX 10
#define BLUETOOTH_RX 11
#define BAUD_RATE 9600

//2. 相关对象的初始化(实际使用中只需要初始化头套对象,但是这里为了调试以及控制增加了液晶屏以及蓝牙)

//蓝牙配对参数初始化(测试用)
int sliderVal, button, sliderId;
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth);
//头部装置初始化
HeadGear headGear(PORT_ENA,PORT_IN1,PORT_IN2,PORT_IN3,PORT_IN4,PORT_RIGHT_SHAKER,PORT_LEFT_SHAKER,APORT_RIGHT_PRESS,APORT_LEFT_PRESS);
//液晶屏(测试用)
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI
void draw(float Fr,float Fl,float yaw,float pitch,float roll)
{
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(0,15);
    u8g.print("Fr");
    u8g.setPrintPos(17,15);
    u8g.print(Fr);
    u8g.setPrintPos(50,15);
    u8g.print("Fl");
    u8g.setPrintPos(67,15);
    u8g.print(Fl);
    
    u8g.setPrintPos(0,32);
    u8g.print("yaw");
    u8g.setPrintPos(24,32);
    u8g.print(yaw);
    u8g.setPrintPos(0,48);
    u8g.print("pitch");
    u8g.setPrintPos(42,48);
    u8g.print(pitch);
    u8g.setPrintPos(0,64);
    u8g.print("roll");
    u8g.setPrintPos(36,64);
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
    if(button==0)
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.stop();
    }
    else if(button==1)
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.move();
    }
    else if(button==2)
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.turnRight();
    }
    else if(button==3)
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.turnLeft();
    }
    else if(button==4)
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.vibrate(1,0,0);
    }
    else if(button==5)
    {
        Serial.print("Button: ");
        Serial.println(button);
        headGear.vibrate(0,1,0);
    }
    
    //调节占空比,主要用于调试
    sliderId = phone.getSliderId();
    sliderVal = phone.getSliderVal();
    if (sliderId == 0)
    {
        Serial.print("Slider ID: ");
        Serial.print(sliderId);
        Serial.print("\tValue: ");
        Serial.println(sliderVal);
        
        headGear.setDutyCycle(sliderVal/200.0*255.0);
    }
    
    //屏幕显示
    u8g.firstPage();
    do 
    {
        draw(headGear.getRightForce(),headGear.getLeftForce(),headGear.getYaw(),headGear.getPitch(),headGear.getRoll());
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
