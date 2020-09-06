
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>



//此处为了兼容其他的多串口Arduino板子
#define GpsSerial  Serial
#define DebugSerial Serial
int L = 13; //LED指示灯引脚

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


/*void setup()  //初始化内容
{
  GpsSerial.begin(9600);      //定义波特率9600，和我们店铺的GPS模块输出的波特率一致
  DebugSerial.begin(9600);

  Save_Data.isGetData = false;
  Save_Data.isParseData = false;
  Save_Data.isUsefull = false;
}*/

/*void loop()   //主循环
{
  gpsRead();  //获取GPS数据
  parseGpsBuffer();//解析GPS数据
  getLatitude();
  getLongitude();
  //printGpsBuffer();//输出解析后的数据
}*/

bool outOfChina(double lat, double lon)
{
    if (lon < 72.004 || lon > 137.8347)
        return true;
    if (lat < 0.8293 || lat > 55.8271)
        return true;
    return false;
}


double transformLat(double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}


double transformLon(double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}


void gps_transform( double wgLat, double wgLon, double& mgLat, double& mgLon)
{
if (outOfChina(wgLat, wgLon))
        {
            mgLat = wgLat;
            mgLon = wgLon;
            return;
        }
        double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
        double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);
        double radLat = wgLat / 180.0 * pi;
        double magic = sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
        dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
        mgLat = wgLat + dLat;
        mgLon = wgLon + dLon;
};

int ddmm2dd(const char *ddmm, char *dd)
{
  if (NULL == ddmm || NULL == dd)
  {
    return -1;
  }
  int lenSrc = strlen(ddmm)+1;
  int lenMm = 0;
  int flag = 1;

  memcpy(dd,ddmm,lenSrc);

  char *pcMm;
  double dMm;
  int iMm;

  /* 把pcMm定位到小数点位置 */
  pcMm = strstr(dd,".");

  if (pcMm == NULL) /* 不含小数点的情况 */
  {
    pcMm = dd+strlen(dd)-2;
    iMm = atoi(pcMm);
    dMm = iMm /60.0;
  }
  else /* 含有小数点的情况 */
  {
    /* 有度 */
    if (pcMm - dd > 2)
    {
      pcMm = pcMm - 2;
    }
    else /* 没有度,只有分 */
    {
      pcMm = dd;
      flag = 0;
    }
    /* 将字符串转换为浮点数 */
    dMm = atof(pcMm);
    /* 将分转换为度 */
    dMm /= 60.0;
  }
  /* 把转换后的浮点数转换为字符串 */
  sprintf(pcMm,"%lf",dMm);
  if ( flag )
  {
    /* 去掉小数点前面的0 */
    strcpy(pcMm,pcMm+1);
  }
  /* 保留小数点后6位 */
  pcMm = strstr(dd,".");
  lenMm = strlen(pcMm);
  if ( lenMm > (6+2))
  {
    memset(pcMm+6+2,0,lenMm-6-2);
  }

  return 1;
}

void errorLog(int num)
{
  DebugSerial.print("ERROR");
  DebugSerial.println(num);
  while (1)
  {
    digitalWrite(L, HIGH);
    delay(300);
    digitalWrite(L, LOW);
    delay(300);
  }
}

void printGpsBuffer()
{
  if (Save_Data.isParseData)
  {
    Save_Data.isParseData = false;
    
    DebugSerial.print("Save_Data.UTCTime = ");
    DebugSerial.println(Save_Data.UTCTime);

    if(Save_Data.isUsefull)
    {
      Save_Data.isUsefull = false;
      DebugSerial.print("Save_Data.latitude = ");
      DebugSerial.println(Save_Data.latitude);
      DebugSerial.print("Save_Data.N_S = ");
      DebugSerial.println(Save_Data.N_S);
      DebugSerial.print("Save_Data.longitude = ");
      DebugSerial.println(Save_Data.longitude);
      DebugSerial.print("Save_Data.E_W = ");
      DebugSerial.println(Save_Data.E_W);
    }
    else
    {
      DebugSerial.println("GPS DATA is not usefull!");
    }
    
  }
}

void parseGpsBuffer()
{
  char *subString;
  char *subStringNext;
  if (Save_Data.isGetData)
  {
    Save_Data.isGetData = false;
    //DebugSerial.println("**************");
    //DebugSerial.println(Save_Data.GPS_Buffer);

    
    for (int i = 0 ; i <= 6 ; i++)
    {
      if (i == 0)
      {
        if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
          errorLog(1);  //解析错误
      }
      else
      {
        subString++;
        if ((subStringNext = strstr(subString, ",")) != NULL)
        {
          char usefullBuffer[2]; 
          switch(i)
          {
            case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break; //获取UTC时间
            case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break; //获取UTC时间
            case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;  //获取纬度信息
            case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break; //获取N/S
            case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break; //获取纬度信息
            case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break; //获取E/W

            default:break;
          }

          subString = subStringNext;
          Save_Data.isParseData = true;
          if(usefullBuffer[0] == 'A')
            Save_Data.isUsefull = true;
          else if(usefullBuffer[0] == 'V')
            Save_Data.isUsefull = false;

        }
        else
        {
          errorLog(2);  //解析错误
        }
      }


    }
  }
}


void gpsRead() {
  while (GpsSerial.available())
  {
    gpsRxBuffer[ii++] = GpsSerial.read();
    if (ii == gpsRxBufferLength) clrGpsRxBuffer();
  }

  char* GPS_BufferHead;
  char* GPS_BufferTail;
  if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
  {
    if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
    {
      memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
      Save_Data.isGetData = true;

      clrGpsRxBuffer();
    }
  }
}

void clrGpsRxBuffer(void)
{
  memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
  ii = 0;
}

double getLatitude()
{
   Serial.println("纬度信息：");
   Serial.print("北纬：");
   double gcjla = 0, wsgla = 0, occu1 = 116, occu2 = 0;
   char ddla[32];
   if (Save_Data.latitude != NULL)
   {if(strlen(Save_Data.latitude) > 1)
    {ddmm2dd(Save_Data.latitude, ddla);
    wsgla = atof(ddla);}
   //gps_transform(wsgla, occu1, gcjla, occu2);
   if (wsgla != 0)
   {
    gps_transform(wsgla, occu1, gcjla, occu2);
   }
   }
   else 
    {gcjla = 0;}
   Serial.println(gcjla);
   return gcjla;
}

double getLongitude()
{
  Serial.println("经度信息：");
  Serial.print("东经：");
  double gcjlo = 0, wsglo = 0, occu1 = 40, occu2 = 0;
  char ddlo[32];
  if (Save_Data.longitude != NULL)
  {
    if (strlen(Save_Data.longitude) > 1)
    {ddmm2dd(Save_Data.longitude, ddlo);
    wsglo = atof(ddlo);}
   //gps_transform(occu1, wsglo, occu2, gcjlo);
   if (wsglo != 0)
   {
    gps_transform(occu1, wsglo, occu2, gcjlo);
   }
   
    }
  else 
    {gcjlo = 0;}
  Serial.println(gcjlo);
  return gcjlo;
}