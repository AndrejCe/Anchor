#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//Подключаем библиотеку I2C.
#include <Wire.h>
///////////////////////////////////////////////////////////
//Подключаю функции управления моторами
#include "motor.h"
//Подключаю функции HMC5883L
#include "compas.h"
static const uint32_t GPSBaud = 9600;
// 
/*
-2 Motor Control Board driven by Arduino.

Connection to the -2 board:
 pin 1 (RPWM) to Arduino pin 5(PWM)
 pin 2 (LPWM) to Arduino pin 6(PWM)
 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
 pin 8 (GND) to Arduino GND
 pins 5 (R_IS) and 6 (L_IS) not connected
Pin E to D13
*/

 
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

  // The TinyGPS++ object
  TinyGPSPlus gps;
  int dp_Switch = 0;
  int force_motor1 = 0;
  int force_motor2 = 0;
  int dP_start = 0;
  int data = 0;  
  int windSensorValue  = 0;
  float dPpoint_Lat = 0 ;
  float dPpoint_Lng = 0 ;

 void setup()
   {
  Serial.begin(9600);
  Serial1.begin(GPSBaud);
  Serial3.begin(9600);
    //Запускаем связь по шине I2C.
  Wire.begin();
  //Компас
  compas_setup();      // инициализация HMC5883L
  //Для анализа окончания процесса будем использовать светодиод.
  pinMode(13, OUTPUT);// DP start indication
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
    }

void loop()
{ 
  // heading description: -180 .....  180
  
    float heading = get_Compass_ang() ;
    if (heading < (-168.5)){
      heading = -180;
    }
    if (heading > 168.5){
      heading = 180;
    }
     heading = int(heading/22.5) * 22.5 ;
     
             //  GPS coordinates.
        
  float present_Lat = float(gps.location.lat()) ;
  float present_Lng = float(gps.location.lng()) ;
    
  Serial3.print("heading=") ;
  Serial3.println(heading) ;
  Serial3.print("Sats: ");
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  Serial3.println();
  Serial3.println("lat:                       lng: ");
  Serial3.print(present_Lat);
  Serial3.print("       ");
  Serial3.println(present_Lng);
  
   smartDelay(1000);
    if (millis() > 1000 && gps.charsProcessed() < 10)
   { 
    Serial3.println(F("No GPS data received: check wiring"));
  }
  // .......................


      
  // read serial DP command
  
  data = Serial3.read();
          Serial3.print("Data from BT");
         Serial3.println(data);
         
  // DP start/stop 
  
  if (data == 68){    // DP started
    dp_Switch = 1;
     digitalWrite(13, HIGH);
                 }
                 
  if (data == 48){    // DP stop 
    dp_Switch = 0;
    force_motor1 = 0;
    force_motor2 = 0;
    dP_start = 0;
                 }
                 
    if ( dp_Switch == 1)   {        // DP point started for
          if (dP_start == 0) { 
     dPpoint_Lat = present_Lat;
     dPpoint_Lng = present_Lat;
     dP_start = 1;
                             }   
        
      windSensorValue = 0;           // read  value from the Wind sensor:

      unsigned long distanceKmToDPpoint =
      (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      dPpoint_Lat, 
      dPpoint_Lng) ;
      Serial.print("Distance to DP point: ");
      printInt(distanceKmToDPpoint, gps.location.isValid(), 9);

  double courseToDPpoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      dPpoint_Lat, 
      dPpoint_Lng);
   
    printFloat(courseToDPpoint, gps.location.isValid(), 7, 2);
    float courseToDP = courseToDPpoint;
    int distanceKmToDP = distanceKmToDPpoint;
    if ( distanceKmToDP > 10  )
    {
         distanceKmToDP = 10;
    }
        if (distanceKmToDP < -10  )
    {
         distanceKmToDP = - 10;
    }
    if ( courseToDP > 0 && courseToDP < 90 )
    {
     int force_motor1 =   sin(heading) * distanceKmToDP;            // force motor 1 between -10 ... 10
     int force_motor2 = - sin(heading - 45 )* distanceKmToDPpoint;  // force motor 2 between -10 ... 10
    }
    if (courseToDP>89 && courseToDP<180)
    {
     int force_motor1 =  sin(heading - 45 ) * distanceKmToDP;       // force motor 1 between -10 ... 10
     int force_motor2 = - sin(heading + 45 )* distanceKmToDP;       // force motor 2 between -10 ... 10
    }
    if (courseToDP > 179 && courseToDP < 270 )
    {
     int force_motor1 = sin(heading - 135 ) * distanceKmToDP;       // force motor 1 between -10 ... 10
     int force_motor2 = sin(heading - 45 ) * distanceKmToDP;        // force motor 2 between -10 ... 10
    }
        if (courseToDP > 269 && courseToDP < 0)
    {
     int force_motor1 = - sin(heading - 45 )* distanceKmToDP;       // force motor 1 between -10 ... 10
     int force_motor2 =  sin(heading - 135 )* distanceKmToDP;       // force motor 2 between -10 ... 10
    }
    
  Serial3.println();
  Serial3.println("DP STARTED ");      
  Serial3.println("DPpoint_Lat:                   DPpoint_Lat: ");
  Serial3.print(dPpoint_Lat);
  Serial3.print("       ");
  Serial3.println(dPpoint_Lng);
                             
      if ( windSensorValue < 1 ){   // keep heading for DP 
         force_motor2 =  -1 ;       // keep heading for DP 
                                }   // keep heading for DP 
      if ( windSensorValue > 1 ){   // keep heading for DP 
         force_motor2 =   1  ;      // keep heading for DP 
                                }   // keep heading for DP                      
    }                // end of if ( dp_Switch == 1){ 
    
                // DP stop
  
     if   ( dp_Switch == 0 )  {
          force_motor1 = 0 ;
          force_motor2 = 0 ;
          Serial3.println("DP STOP  ");
          digitalWrite(13, LOW);
                              }
                              
         // PWM start for motor 1
         // force_motor1 is in the range -10 to 10
     
     force_motor1 = ( force_motor1 + 10 ) * 51.2  ;
     
     // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (force_motor1 < 512)
  {
    // reverse rotation
    int force_motor1 = -(force_motor1 - 511) / 2;
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, force_motor1);
  }
  else
  {
    // forward rotation
    int force_motor1 = (force_motor1 - 512) / 2;
    analogWrite(LPWM_Output, force_motor1);
    analogWrite(RPWM_Output, 0);
  }

   // PWM start for motor 2
 
     // force_motor1 is in the range -10 to 10
     
     force_motor2 = (force_motor2+ 10) * 51.2 ;
     
     // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (force_motor2 < 512)
  {
    // reverse rotation
    int force_motor2 = -(force_motor2 - 511) / 2;
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, force_motor2);
  }
  else
  {
    // forward rotation
    int force_motor2 = (force_motor2 - 512) / 2;
    analogWrite(LPWM_Output, force_motor2);
    analogWrite(RPWM_Output, 0);
  }

   Serial3.print("Force Motor 1:  ");
   Serial3.print(force_motor1);
   Serial3.print(F("   Force Motor2:  "));
   Serial3.println(force_motor2);
   Serial3.println();
   Serial.println(heading); 
    delay(3000);
  
       }   // end of  void loop()    
       
  //..................................................................... 
  
// This custom version of delay() ensures that the gps object
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
    Serial3.print('*');
    Serial3.print(' ');
  }
  else
  {
    Serial3.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial3.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial3.print(sz);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial3.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
