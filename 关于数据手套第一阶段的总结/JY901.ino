#include <Wire.h>
#include <JY901.h>
/*
Test on Uno R3.
JY901    UnoR3
SDA <---> SDA A4
SCL <---> SCL A5
*/
unsigned long time1 = 0;
const int selectPins[6] = {2, 3, 4, 5, 6, 7}; 
void setup() 
{
  Serial.begin(38400);  
  for (int i=0; i<6; i++)  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
  }
  JY901.StartIIC();
  Serial.print(",");
}
void loop() 
{
  //下面的代码是选择读取传感器的端口。如果没使用复用器，则不需要加。
  selectMuxPin(5);
  selectMuxPin2(3);
//  time1 = micros();
//  Serial.print(time1); Serial.print(",");
  JY901.GetAngle();
  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.println((float)JY901.stcAngle.Angle[2]/32768*180,5);

//  //选择第二个传感器
//  selectMuxPin(1);
//  JY901.GetAngle();
//  Serial.print(",");
//  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);
//
////  //选择第三个传感器
//  selectMuxPin(2);
//  JY901.GetAngle();
//  Serial.print(",");
//  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);
//
////  //选择第四个传感器
//  selectMuxPin(3);
//  JY901.GetAngle();
//  Serial.print(",");
//  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[2]/32768*180,5);
//
//  //  //选择第五个传感器
//  selectMuxPin(4);
//  JY901.GetAngle();
//  Serial.print(",");
//  Serial.print((float)JY901.stcAngle.Angle[0]/32768*180,5);Serial.print(",");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180,5);Serial.print(",");Serial.println((float)JY901.stcAngle.Angle[2]/32768*180,5);
////  JY901.GetGyro();
//  Serial.print((float)JY901.stcGyro.w[0]/32768*2000,5);Serial.print(",");Serial.print((float)JY901.stcGyro.w[1]/32768*2000,5);Serial.print(",");Serial.println((float)JY901.stcGyro.w[2]/32768*2000,5);
  //  Serial.println("");
  //  delay(500);
}
void selectMuxPin(byte pin)
{
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}
void selectMuxPin2(byte pin)
{
  for (int i=3; i<6; i++)
  {
    if (pin & (1<<i-3))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}

