#include "U8glib.h"
#include <ArduinoBlue.h>//蓝牙依赖
#include <SoftwareSerial.h>

//蓝牙端口(仅测试用)
#define BLUETOOTH_TX 10
#define BLUETOOTH_RX 11
#define BAUD_RATE 9600

//蓝牙配对参数初始化(测试用)
int sliderVal, button, sliderId;
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth);

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
    
    // u8g.setPrintPos(0,32);
    // u8g.print("yaw");
    // u8g.setPrintPos(24,32);
    // u8g.print(yaw);
    // u8g.setPrintPos(0,48);
    // u8g.print("pitch");
    // u8g.setPrintPos(42,48);
    // u8g.print(pitch);
    // u8g.setPrintPos(0,64);
    // u8g.print("roll");
    // u8g.setPrintPos(36,64);
    // u8g.print(roll);
    
}
