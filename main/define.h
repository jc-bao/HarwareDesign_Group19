//1. 关于端口宏的设置

//电机和震动电机端口
#define PORT_ENA 3
#define PORT_IN1 4
#define PORT_IN2 5
#define PORT_IN3 6
#define PORT_IN4 7
#define PORT_RIGHT_SHAKER 12
#define PORT_LEFT_SHAKER 13
#define APORT_RIGHT_PRESS 14
#define APORT_LEFT_PRESS 15

//此处为了兼容其他的多串口Arduino板子
#define GpsSerial  Serial
#define DebugSerial Serial
int L = 13;
// 避障
int TrgPin = A0; 
int EcoPin = A1;
double dist;

const double pi = 3.14159265358979324;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;



struct
{
  char GPS_Buffer[80];
  bool isGetData;   //是否获取到GPS数据
  bool isParseData; //是否解析完成
  char UTCTime[11];   //UTC时间
  char latitude[11];    //纬度
  char N_S[2];    //N/S
  char longitude[12];   //经度
  char E_W[2];    //E/W
  bool isUsefull;   //定位信息是否有效
} Save_Data;

const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int ii = 0;
// HeadGear headGear(PORT_IN1,PORT_IN2,PORT_IN3,PORT_IN4,PORT_IN5,PORT_IN6,PORT_IN7,PORT_IN8,PORT_RIGHT_SHAKER,PORT_LEFT_SHAKER,APORT_RIGHT_FRONT_PRESS,APORT_LEFT_FRONT_PRESS,APORT_RIGHT_BEHIND_PRESS,APORT_LEFT_BEHIND_PRESS);
double destLongitude, destLatitude;
int destNumber;
unsigned char message = '\0';
int len = 0;
int p = 0;
double thisLongitude, thisLatitude;
