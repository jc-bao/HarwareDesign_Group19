//震动时间参数设置
#define SHORT_SHAKE_CYCLE 200 //短震动周期ms
#define SHAKE_TIME 2000 //震动时长
#define MOTOR_TIME 2000 //转头时长

//压力传感器
//传感器电压设置
#define VOLTAGE_MIN 100
#define VOLTAGE_MAX 3300
//压力传感器量程设置
#define PRESS_MIN 0.3f
#define PRESS_MAX 15.0f
//不同位置受力大小设置
//前方受力设置
#define RIGHT_FRONT_FORCE_MIN 0.3f
#define RIGHT_FRONT_FORCE_MAX 5.0f
#define LEFT_FRONT_FORCE_MIN 0.3f
#define LEFT_FRONT_FORCE_MAX 5.0f
//后方受力设置
#define RIGHT_BEHIND_FORCE_MIN 0.3f
#define RIGHT_BEHIND_FORCE_MAX 5.0f
#define LEFT_BEHIND_FORCE_MIN 0.3f
#define LEFT_BEHIND_FORCE_MAX 5.0f
//压力传感器电压设置
#define VOLTAGE_MIN 100
#define VOLTAGE_MAX 3300
//压力传感器量程设置
#define PRESS_MIN 0.3f
#define PRESS_MAX 15.0f

//必须的依赖
#include <Arduino.h>
#include <SoftwareSerial.h>
//陀螺仪依赖
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//调试用的依赖
//蓝牙依赖
#include <ArduinoBlue.h>
#include "U8glib.h"


class HeadGear
{
public:
    /*
     对外接口
     */
    
    //构造函数
    HeadGear(int inputIN1,int inputIN2,int inputIN3,int inputIN4,int inputIN5,int inputIN6,int inputIN7,int inputIN8,int inputRightShaker,int inputLeftShaker,int inputRightFrontPress,int inputLeftFrontPress,int inputRightBehindPress,int inputLeftBehindPress):
    IN1(inputIN1),IN2(inputIN2),IN3(inputIN3),IN4(inputIN4),IN5(inputIN5),IN6(inputIN6),IN7(inputIN7),IN8(inputIN8),RIN(inputRightShaker),LIN(inputLeftShaker),RFPR(inputRightFrontPress),LFPR(inputLeftFrontPress),RBPR(inputRightBehindPress),LBPR(inputLeftBehindPress),
    motorState(0),shakeStartTime(-3000),shakeMode(0),rightState(0),leftState(0),dmpReady(0),mpu(0x68)
    {
    }
    
    //转向类函数
    void turnRight()
    {
        motorState=1;
        motorStartTime=millis();
    }
    void turnLeft()
    {
        motorState=2;
        motorStartTime=millis();
    }
    
    //移动类函数
    void stop()
    {
        vibrate(1,1,1);
    }
    void move()
    {
        vibrate(1,1,0);
    }
    
    //信息获取类函数
    float getYaw()
    {
        return (ypr[0] * 180/M_PI);
    }
    float getPitch()
    {
        return (ypr[1] * 180/M_PI);
    }
    float getRoll()
    {
        return (ypr[2] * 180/M_PI);
    }
    float getRightFrontForce()
    {
        float F=map(analogRead(RFPR), 0, 676, PRESS_MIN, PRESS_MAX);
        return F;
    }
    float getLeftFrontForce()
    {
        float F=map(analogRead(LFPR), 0, 676, PRESS_MIN, PRESS_MAX);
        return F;
    }
    float getRightBehindForce()
    {
        float F=map(analogRead(RBPR), 0, 676, PRESS_MIN, PRESS_MAX);
        return F;
    }
    float getLeftBehindForce()
    {
        float F=map(analogRead(LBPR), 0, 676, PRESS_MIN, PRESS_MAX);
        return F;
    }
    
    //震动调用函数
    //mode:0为短震(用于提示),1为长震(用于警告);其余的1是激活,2是停用
    void vibrate(bool inputRightState,bool inputLeftState,bool inputMode)
    {
        shakeStartTime=millis();
        rightState=inputRightState;
        leftState=inputLeftState;
        shakeMode=inputMode;
    }
    
    /*
     内部接口
     */
    //初始化函数
    void Setup()
    {
        pinMode(IN1,OUTPUT);
        pinMode(IN2,OUTPUT);
        pinMode(IN3,OUTPUT);
        pinMode(IN4,OUTPUT);
        pinMode(IN5,OUTPUT);
        pinMode(IN6,OUTPUT);
        pinMode(IN7,OUTPUT);
        pinMode(IN8,OUTPUT);
        pinMode(RIN,OUTPUT);
        pinMode(LIN,OUTPUT);
        
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,LOW);
        digitalWrite(IN5,LOW);
        digitalWrite(IN6,LOW);
        digitalWrite(IN7,LOW);
        digitalWrite(IN8,LOW);
        digitalWrite(RIN,LOW);
        digitalWrite(LIN,LOW);

        //初始化MPU
        // join I2C bus (I2Cdev library doesn't do this automatically)
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
            Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif
        mpu.initialize();
        devStatus = mpu.dmpInitialize();
        //设置陀螺仪
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);
        //校准
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;
        //获取DMP包
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    //更新函数
    void Update()
    {
        //更新电机状态
        if((millis()-motorStartTime)<MOTOR_TIME && (millis()-motorStartTime)>0)
        {
            if(motorState==1)
            {
                //调整右前方电机
                if(getRightFrontForce()<RIGHT_FRONT_FORCE_MAX)
                {
                    digitalWrite(IN1,HIGH);
                    digitalWrite(IN2,LOW);
                }
                else
                {
                    digitalWrite(IN1,HIGH);
                    digitalWrite(IN2,HIGH);
                }
                //调整左前方电机
                if(getLeftFrontForce()>LEFT_FRONT_FORCE_MIN)
                {
                    digitalWrite(IN3,LOW);
                    digitalWrite(IN4,HIGH);
                }
                else
                {
                    digitalWrite(IN3,HIGH);
                    digitalWrite(IN4,HIGH);
                }
                //调整右后方电机
                if(getRightBehindForce()<RIGHT_BEHIND_FORCE_MAX)
                {
                    digitalWrite(IN5,HIGH);
                    digitalWrite(IN6,LOW);
                }
                else
                {
                    digitalWrite(IN5,HIGH);
                    digitalWrite(IN6,HIGH);
                }
                //调整左后方电机
                if(getLeftBehindForce()<LEFT_BEHIND_FORCE_MAX)
                {
                    digitalWrite(IN7,HIGH);
                    digitalWrite(IN8,LOW);
                }
                else
                {
                    digitalWrite(IN7,HIGH);
                    digitalWrite(IN8,HIGH);
                }
            }
            else if(motorState==2)
            {
                //调整右前方电机
                if(getRightFrontForce()>RIGHT_FRONT_FORCE_MIN)
                {
                    digitalWrite(IN1,LOW);
                    digitalWrite(IN2,HIGH);
                }
                else
                {
                    digitalWrite(IN1,HIGH);
                    digitalWrite(IN2,HIGH);
                }
                //调整左前方电机
                if(getLeftFrontForce()<LEFT_FRONT_FORCE_MAX)
                {
                    digitalWrite(IN3,HIGH);
                    digitalWrite(IN4,LOW);
                }
                else
                {
                    digitalWrite(IN3,HIGH);
                    digitalWrite(IN4,HIGH);
                }
                //调整右后方电机
                if(getRightBehindForce()<RIGHT_BEHIND_FORCE_MAX)
                {
                    digitalWrite(IN5,HIGH);
                    digitalWrite(IN6,LOW);
                }
                else
                {
                    digitalWrite(IN5,HIGH);
                    digitalWrite(IN6,HIGH);
                }
                //调整左后方电机
                if(getLeftBehindForce()<LEFT_BEHIND_FORCE_MAX)
                {
                    digitalWrite(IN7,HIGH);
                    digitalWrite(IN8,LOW);
                }
                else
                {
                    digitalWrite(IN7,HIGH);
                    digitalWrite(IN8,HIGH);
                }
            }
            else if(motorState==0)
            {
                digitalWrite(IN1,HIGH);
                digitalWrite(IN2,HIGH);
                digitalWrite(IN3,HIGH);
                digitalWrite(IN4,HIGH);
                digitalWrite(IN5,HIGH);
                digitalWrite(IN6,HIGH);
                digitalWrite(IN7,HIGH);
                digitalWrite(IN8,HIGH);
            }
        }
        else
        {
            //调整右前方电机
            if(getRightFrontForce()>RIGHT_FRONT_FORCE_MIN)
            {
                digitalWrite(IN1,LOW);
                digitalWrite(IN2,HIGH);
            }
            else
            {
                digitalWrite(IN1,HIGH);
                digitalWrite(IN2,HIGH);
            }
            //调整左前方电机
            if(getLeftFrontForce()>LEFT_FRONT_FORCE_MIN)
            {
                digitalWrite(IN3,LOW);
                digitalWrite(IN4,HIGH);
            }
            else
            {
                digitalWrite(IN3,HIGH);
                digitalWrite(IN4,HIGH);
            }
            //调整右后方电机
            if(getRightBehindForce()>RIGHT_BEHIND_FORCE_MIN)
            {
                digitalWrite(IN5,LOW);
                digitalWrite(IN6,HIGH);
            }
            else
            {
                digitalWrite(IN5,HIGH);
                digitalWrite(IN6,HIGH);
            }
            //调整左后方电机
            if(getLeftBehindForce()>LEFT_BEHIND_FORCE_MIN)
            {
                digitalWrite(IN7,LOW);
                digitalWrite(IN8,HIGH);
            }
            else
            {
                digitalWrite(IN7,HIGH);
                digitalWrite(IN8,HIGH);
            }
        }
        
        //更新震动器状态
        if((millis()-shakeStartTime)<SHAKE_TIME && (millis()-shakeStartTime)>0)
        {
            if(rightState)
            {
                //长震动模式
                if(shakeMode)
                {
                    digitalWrite(PORT_RIGHT_SHAKER,HIGH);
                }
                //短震动模式
                else
                {
                    if( ( (int)((millis()-shakeStartTime)/SHORT_SHAKE_CYCLE) % 2 ) == 0 )
                        digitalWrite(PORT_RIGHT_SHAKER,HIGH);
                    else
                        digitalWrite(PORT_RIGHT_SHAKER,LOW);
                }
            }
            if(leftState)
            {
                //长震动模式
                if(shakeMode)
                {
                    digitalWrite(PORT_LEFT_SHAKER,HIGH);
                }
                //短震动模式
                else
                {
                    if( ( (int)((millis()-shakeStartTime)/SHORT_SHAKE_CYCLE) % 2 ) == 0 )
                        digitalWrite(PORT_LEFT_SHAKER,HIGH);
                    else
                        digitalWrite(PORT_LEFT_SHAKER,LOW);
                }
            }
        }
        else
        {
            digitalWrite(PORT_RIGHT_SHAKER,LOW);
            digitalWrite(PORT_LEFT_SHAKER,LOW);
        }
        
        //更新角度
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
    }
    
    //电机控制类
    void setMotorState(int inputState)
    {
        motorState=inputState;
    }
    
protected:
    
private:
    //下面是关于MPU6050的使用,我看不懂
    MPU6050 mpu;//mpu对象,陀螺仪与加速度计
    // MPU control/status vars
    bool dmpReady;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    
    
    int IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8;//电机控制
    int RIN,LIN;//震动马达
    int RFPR,LFPR,RBPR,LBPR;//压力传感器
    
    int motorState;//记录电机状态 0-不动 1-正转 2-反转
    
    //记录震动开始时间用于判断时长以及是否震动,初始值为-3000,不震动,若发现现在时间和开始时间相差1500(1.5s),则按照模式震动
    unsigned long shakeStartTime;
    bool shakeMode;
    bool rightState;//电机是否震动状态
    bool leftState;
    
    //记录电机开始时间用于判断时长以及是否使用电机,初始值为-3000,不通电,若发现现在时间和开始时间相差2000(2s),则按照模式启动电机
    unsigned long motorStartTime;
};