#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass;
#define EATH_RADIUS 6372795
///////////////////////////////////////////////////////////
// The TinyGPS++ object
TinyGPSPlus gps;
int dp_Switch = 0;
float force_motor1 = 0 ;
float force_motor2 = 0 ;
float previos_force_motor1 = 512 ;
float previos_force_motor2 = 512 ;
int dP_start = 0;
int data = 0;
long dPpoint_Lat;
long dPpoint_Lng;
long dPpoint_Lat1;
long dPpoint_Lng1;
int distanceKmToDP;
float heading = 0;
float my_heading = 0;
long present_Lat = 54253565 ;
long present_Lng = 24556752 ;


void setup()
{
  Serial.begin(9600);
while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

    if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }
  //Для анализа окончания процесса будем использовать светодиод.
  pinMode(13, OUTPUT); // DP start indication
  pinMode(2, OUTPUT); 
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT);  
  pinMode(6, OUTPUT);  
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(9, OUTPUT);  
  pinMode(10, OUTPUT);  
}

void loop()
{
 int  func_s = count_s(heading , distanceKmToDP);
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }

  if (heading > 2 * PI){
    heading -= 2 * PI;
  }

  // Convert to degrees
  
     heading = my_heading ;                                 //   TEST
  // heading = heading * 180/PI;                            //   TEST  
    if (data == 65)  {        // "A" PRESSED = heading + 5  //   TEST
   my_heading = my_heading + 5;                             //   TEST                                       
   // dPpoint_Lat1 = present_Lat;                           //   TEST
   // dPpoint_Lng1 = present_Lng;                           //   TEST
                     }                                      //   TEST
if (data == 67 )
               {                                            // "C" PRESSED = heading - 5
  my_heading = my_heading - 5;                              //   TEST     
               }                                            //   TEST 

heading = my_heading;                                       //   TEST 

  Serial.println();  
  Serial.print(" Heading = ");
  Serial.println(heading);

  delay(1000);

  // .......................

  // read serial DP command

  data = Serial.read();
  Serial.print("Data BT  ");
  Serial.println(data);


  // DP start/stop

  if (data == 68)            // DP started    ____ "S" pressed
                   {         // DP started
    dp_Switch = 1;           // DP started
    digitalWrite(13, HIGH);  // DP started
                   }         // DP started

  if (data == 48)         // DP stop          _____"O" pressed
                   {      // DP stop
    dp_Switch = 0;        // DP stop
    force_motor1 = 0;     // DP stop
    force_motor2 = 0;     // DP stop
    dP_start = 0;         // DP stop
                   }      // DP stop

  if ( dp_Switch == 1)  
{                            // DP point started 
         if (dP_start == 0)
             {
 dPpoint_Lat = present_Lat ;
 dPpoint_Lng = present_Lng ;
  
      dP_start = 1;
             } 
   //          else
   //      {
  //  if (data == 65)             // go to  DP1      __ "A" pressed
   //          {                 // go to  DP1
   //   dPpoint_Lat = dPpoint_Lat1;
   //   dPpoint_Lng = dPpoint_Lng1;
   //          }
    //     }
    Serial.println(present_Lat  - dPpoint_Lat );
    Serial.println(present_Lng  - dPpoint_Lng );
    
    unsigned long distanceKmToDP =
      (unsigned long)TinyGPSPlus::distanceBetween(
        present_Lat/10,
        present_Lng/10,
        dPpoint_Lat/10,
        dPpoint_Lng/10)/24750 ;
        
  double courseToDp =
    TinyGPSPlus::courseTo(
        dPpoint_Lat, 
        dPpoint_Lng,
        present_Lat,
        present_Lng) + 90 ;

      if (courseToDp > 359)
      {
        courseToDp = courseToDp - 360 ;
      }
    Serial.print("Heading  ");
    Serial.println(heading);

    if ( distanceKmToDP > 10  )
    {
      float distanceKmToDP = 10;
    }
    if (distanceKmToDP < -10  )
    {
      float distanceKmToDP = - 10;
    }

    Serial.print("distanceKmToDP  ");
    Serial.println(distanceKmToDP);
    Serial.print("courseToDp  ");
    Serial.println(courseToDp);
        if (data == 78) // start Lat + 1__ "N" pressed
    {
          present_Lat =  present_Lat + 1;
    }
    if (data == 77) // start Lat - 1__ "M" pressed
    {
      present_Lat =  present_Lat - 1;    
    }
    if (data == 75)  //  start Lng + 1__ "K" pressed
    {
       present_Lng =  present_Lng + 1; 
    }
    else
    {     
    if (data == 76) // start Lng - 1 __ "L" pressed
    {
        present_Lng =  present_Lng - 1;
    }
    }
    if ( courseToDp > -1  )
    {
      if ( courseToDp < 90)
      {
        force_motor1 = - count_s(heading,distanceKmToDP) ;                          // force motor 1 between -10 ... 10
        force_motor2 = - count_so( heading , courseToDp , distanceKmToDP )  ;  
         Serial.print("courseToDp0...90  ");
         // force motor 2 between -10 ... 10
      }
    }
    if ( courseToDp > 89  )
    {
      if ( courseToDp < 180 )
      {
        force_motor1 = count_s(heading,distanceKmToDP) ;                          // force motor 1 between -10 ... 10
        force_motor2 =  count_so( heading , courseToDp , distanceKmToDP ) ;                              // force motor 2 between -10 ... 10
        Serial.println("courseToDp90...180  ");
      }
    }
    if ( courseToDp > 179  )
    {     
      if ( courseToDp < 270 )
        {
        force_motor1 = count_s(heading,distanceKmToDP) ;                            // force motor 1 between -10 ... 10
        force_motor2 =  count_so( heading , courseToDp , distanceKmToDP ) ;                             // force motor 2 between -10 ... 10
       Serial.print("courseToDp180...270  ");
        }
    }
    if ( courseToDp > 269 )
       {    
      if (  courseToDp < 361 )
      {
        force_motor1 =  count_s(heading,distanceKmToDP) ;            // force motor 1 between -10 ... 10
        force_motor2 =  count_so( heading , courseToDp , distanceKmToDP ) ;         // force motor 2 between -10 ... 10
       Serial.print("courseToDp270...360  ");
      }
       }
    
  }   // end of if ( dp_Switch == 1)
  

             
  if   ( dp_Switch == 0 )  // DP stop
    {
    force_motor1 = 0 ;
    force_motor2 = 0 ;
    Serial.println("DP STOP  ");
    digitalWrite(13, LOW);
    }
  // PWM start for motor 1
  // force_motor1 is in the range -10 to 10
  force_motor1 = ( force_motor1 + 10 ) * 25.6 + 256 ;   // force motor 1 between 0...1024 with 0  =  512

  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (force_motor1 < 390)
  {
    force_motor1 = 390 ;
  } 
  else 
          {
  if (force_motor1 > 650)
  {
    force_motor1 = 650 ;
  } 
          }     
    force_motor1 = (force_motor1 + previos_force_motor1) / 2;
  previos_force_motor1 = force_motor1   ;

  if (force_motor1 > 512)  
  {                                   
  digitalWrite(4, LOW);     // AS 100%
  digitalWrite(6, LOW);     // AS 50%
  digitalWrite(7, HIGH);    // AH 50%
  digitalWrite(9, LOW);     // AH 100%
  }
    if (force_motor1 > 612)  
  {
  digitalWrite(4, LOW);     // AS 100%
  digitalWrite(6, LOW);     // AS 50%
  digitalWrite(7, LOW);     // AH 50%
  digitalWrite(9, HIGH);    // AH 100%
  }         
  if (force_motor1 < 512)  
  {                                  
  digitalWrite(4, LOW);     // AS 100%
  digitalWrite(6, HIGH);    // AS 50%
  digitalWrite(7, LOW);     // AH 50%
  digitalWrite(9, LOW);     // AH 100%
  }
    if (force_motor1 < 400)
    { 
  digitalWrite(4, HIGH);   // AS 100%
  digitalWrite(6, LOW);    // AS 50%
  digitalWrite(7, LOW);    // AH 50%
  digitalWrite(9, LOW);    // AH 100%
    }          
          // PWM start for motor 2
  force_motor2 = ( force_motor2 + 10 ) * 25.6 + 256 ;    // force motor 2 between 0...1024 with 0  =  512
  if (force_motor2 < 290)
  {
    force_motor2 = 290 ;
  }  
  else
  {
  if (force_motor2 > 750)
  {
    force_motor2 = 750 ;
  }
  }


  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  
  
    force_motor2 = (force_motor2 + previos_force_motor2)/2;
    previos_force_motor2 = force_motor2 ;

 
  if (force_motor2 > 512)  
  {                                   // Forward rotation
  digitalWrite(2, LOW);     //  PS 50%    
  digitalWrite(5, HIGH);    //SBS 50%
  digitalWrite(8, LOW);     //  PS 100%
  digitalWrite(10, LOW);    //SBS 100%
  }
    if (force_motor2 > 612)
    {
  digitalWrite(2, LOW);    //  PS 50%    
  digitalWrite(5, LOW);    //SBS 50%
  digitalWrite(8, LOW);    //  PS 100%
  digitalWrite(10, HIGH);  //SBS 100%
  }      
  if (force_motor2 < 512)  
  {                              
  digitalWrite(2, HIGH);    //  PS 50%    
  digitalWrite(5, LOW);     //SBS 50%
  digitalWrite(8, LOW);     //  PS 100%
  digitalWrite(10, LOW);    //SBS 100%
  }
    if (force_motor2 < 400)
    {  
  digitalWrite(2, LOW);     //  PS 50% 
  digitalWrite(5, LOW);     //SBS 50%
  digitalWrite(8, HIGH);    //  PS 100%
  digitalWrite(10, LOW);    //SBS 100%
    }
        if (force_motor1 > 485)  
  {        
     if (force_motor1 < 539) 
     {                        
  digitalWrite(4, LOW);    // AS 100%
  digitalWrite(6, LOW);    // AS 50%
  digitalWrite(7, LOW);    // AH 50%
  digitalWrite(9, LOW);    // AH 100%
     } 
  }
    if (force_motor2 > 489)  
  {        
     if (force_motor2 < 538) 
     {                        
  digitalWrite(2, LOW);    //  PS 50%    
  digitalWrite(5, LOW);    //  SBS 50%
  digitalWrite(8, LOW);    //  PS 100%
  digitalWrite(10, LOW);   //  SBS 100%
    } 
  }
   Serial.print("Force Motor 1:  ");
   Serial.println(force_motor1);
   Serial.print("Force Motor2:  ");
   Serial.println(force_motor2); 
}                                                                            // end of  void loop()

//-----------------------------------------

int count_s( float heading , int distanceKmToDP )
{
  int result;
  result = sin(( heading + 45 )/ (180/PI )) * distanceKmToDP  ;
 Serial.println(result); 
  return  result;
}

int count_so( float heading , int courseToDp , int distanceKmToDP )
{
  int result;
  result = sin(( heading - courseToDp)/( 180/PI)) * distanceKmToDP ;
  Serial.println(result);
  return  result;
}
