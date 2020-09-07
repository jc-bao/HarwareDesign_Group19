//相关参数的设置
#define SHORT_SHAKE_CYCLE 200 //短震动周期ms
#define SHAKE_TIME 2000 //震动时长
#define MOTOR_TIME 2000 //转头时长

//压力传感器(这一部分是是为了兼容性)
#define PRESS_MIN 0.3f
#define PRESS_MAX 15.0f
#define VOLTAGE_MIN 100
#define VOLTAGE_MAX 3300

//MPU6050
//#define OUTPUT_READABLE_YAWPITCHROLL //输出欧氏角

#include <Arduino.h>


class HeadGear
{
public:
    /*
    对外接口
     */
    
    //构造函数
    HeadGear(int inputENA,int inputIN1,int inputIN2,int inputIN3,int inputIN4,int inputRightShaker,int inputLeftShaker,int inputRightPress,int inputLeftPress):
    ENA(inputENA),IN1(inputIN1),IN2(inputIN2),IN3(inputIN3),IN4(inputIN4),RIN(inputRightShaker),LIN(inputLeftShaker),RPR(inputRightPress),LPR(inputLeftPress),
    dutyCycle(255),motorState(0),shakeStartTime(-3000),shakeMode(0),rightState(0),leftState(0)
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
    float getRightForce()
    {
        float Fr=map(analogRead(RPR), 0, 676, PRESS_MIN, PRESS_MAX);
        return 0;
    }
    float getLeftForce()
    {
        float Fl=map(analogRead(LPR), 0, 676, PRESS_MIN, PRESS_MAX);
        return 0;
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
        pinMode(RIN,OUTPUT);
        pinMode(LIN,OUTPUT);
        
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,LOW);
        digitalWrite(RIN,LOW);
        digitalWrite(LIN,LOW);

    }
    
    //更新函数
    void Update()
    {
        //更新占空比
        analogWrite(PORT_ENA,dutyCycle);
        
        //更新电机状态
        if( ( (millis()-motorStartTime)<MOTOR_TIME ) && (millis()-motorStartTime)>0)
        {
            if(motorState==1)
            {
                    digitalWrite(IN1,HIGH);
                    digitalWrite(IN2,LOW);
                    digitalWrite(IN3,HIGH);
                    digitalWrite(IN4,LOW);
            }
            else if(motorState==2)
            {
                digitalWrite(IN1,LOW);
                digitalWrite(IN2,HIGH);
                digitalWrite(IN3,LOW);
                digitalWrite(IN4,HIGH);
            }
            else if(motorState==0)
            {
                digitalWrite(IN1,HIGH);
                digitalWrite(IN2,HIGH);
                digitalWrite(IN3,HIGH);
                digitalWrite(IN4,HIGH);
            }
        }
        else
        {
            digitalWrite(IN1,HIGH);
            digitalWrite(IN2,HIGH);
            digitalWrite(IN3,HIGH);
            digitalWrite(IN4,HIGH);
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
        
    }
    
    //电机控制类
    void setDutyCycle(int inputDutyCycle)
    {
        dutyCycle=inputDutyCycle;
    }
    int getDutyCycle()
    {
        return dutyCycle;
    }
    void setMotorState(int inputState)
    {
        motorState=inputState;
    }
    
protected:
    
private:
    int ENA,IN1,IN2,IN3,IN4;//电机控制
    int RIN,LIN;//震动马达
    int RPR,LPR;//压力传感器
    
    int dutyCycle;//记录占空比
    int motorState;//记录电机状态 0-不动 1-正转 2-反转
    
    //记录震动开始时间用于判断时长以及是否震动,初始值为-3000,不震动,若发现现在时间和开始时间相差1500(1.5s),则按照模式震动
    unsigned long shakeStartTime;
    bool shakeMode;
    bool rightState;//电机是否震动状态
    bool leftState;
    
    //记录电机开始时间用于判断时长以及是否使用电机,初始值为-3000,不通电,若发现现在时间和开始时间相差2000(2s),则按照模式启动电机
    unsigned long motorStartTime;
};
