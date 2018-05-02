//IMU_901 Data Glove
#include <Wire.h>
#include <JY901.h>
#include <math.h>
/*
 * 其中数据手套的连线
 * 大拇指的灰紫蓝  连上234，小拇指的灰紫蓝连上567
 * 黄色连A4，绿色连A5
 */
 struct Data{
  float temp1;
  float temp2;
};
const int selectPins[6] = {2, 3, 4, 5, 6, 7};
//下面变量的定义为传感器获得的值，其中关节的顺序为 手掌 大拇指 食指 中指 无名指 小拇指
float palmx,palmy,palmz;
float bigjy,bigjz,bigzy,bigzz;
float indexjy,indexjz,indexzy,indexzz;
float midjy,midjz,midzy,midzz;
float ringjy,ringjz,ringzy,ringzz;
float litjy,litjz,litzy,litzz;

//下面变量的定义为给blender的关节角度值
float bpalmx,bpalmy,bpalmz;
float bbigjy,bbigjz,bbigzy,bbigzz;
float bindexjy,bindexjz,bindexzy,bindexzz,bindexyy;
float bmidjy,bmidjz,bmidzy,bmidzz,bmidyy;
float bringjy,bringjz,bringzy,bringzz,bringyy;
float blitjy,blitjz,blitzy,blitzz,blityy;
//z轴的每个手指的角度偏差值
float sum2,sum3,sum4,sum5;
void setup() 
{
  Serial.begin(57600); // Initialize the serial port
  // Set up the select pins as outputs:
  for (int i=0; i<6; i++)  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
  }
  JY901.StartIIC();
  //角度初始化
  for(int i = 0;i<100;i++){
      selectMuxPin(5);
      selectMuxPin2(0);
      JY901.GetAngle();
      palmz = (float)JY901.stcAngle.Angle[2]/32768*180;
      //选择食指近指
      selectMuxPin(2);
      JY901.GetAngle();
      indexjz = (float)JY901.stcAngle.Angle[2]/32768*180;
      //选择中指近指
      selectMuxPin(5);
      selectMuxPin2(1);
      JY901.GetAngle();
      midjz = (float)JY901.stcAngle.Angle[2]/32768*180;
      //选择无名指近指
      selectMuxPin(5);
      selectMuxPin2(2);
      JY901.GetAngle();
      ringjz = (float)JY901.stcAngle.Angle[2]/32768*180;
      //选择小拇指近指
      selectMuxPin(5);
      selectMuxPin2(4);
      JY901.GetAngle();
      litjz = (float)JY901.stcAngle.Angle[2]/32768*180;
      
      sum2 = sum2 + (palmz - indexjy); //食指
      sum3 = sum3 + (palmz - midjz);
      sum4 = sum4 + (palmz - ringjz);
      sum5 = sum5 + (palmz - litjz); //小拇指
    }
    sum2 = sum2/100.0;
    sum3 = sum3/100.0;
    sum4 = sum4/100.0;
    sum5 = sum5/100.0;
}
void loop() {
  // put your main code here, to run repeatedly:
  //使用的blender模型中都是XYZ欧拉角度，如果不是，请将关节进行更改。
  //其中，在对blender传输数据时
  //其中传感器的 y 轴数据对应模型的 euler[0],也就是关节主要运动的关节。关节和父关节之间的角度差为正值。PIP
  //             x 轴数据 主要控制  euler[1]，控制关节本身自己的旋转，这个数据可以不接受
  //             z 轴数据对应模型的 euler[2]，控制MP关节的运动。MP,只有近指才会发生。
  //blender模型传递子类和父类的角度差距数据，不是直接读取的传感器数据
  //选择主传感器，手掌传感器
  selectMuxPin(5);
  selectMuxPin2(0);
  JY901.GetAngle();
  palmx = (float)JY901.stcAngle.Angle[0]/32768*180;
  palmy = (float)JY901.stcAngle.Angle[1]/32768*180;
  palmz = (float)JY901.stcAngle.Angle[2]/32768*180;

  //大拇指
  selectMuxPin(0);
  JY901.GetAngle();
  bigjy = (float)JY901.stcAngle.Angle[1]/32768*180;
  bigjz = (float)JY901.stcAngle.Angle[2]/32768*180;
  selectMuxPin(1);
  JY901.GetAngle();
  bigzy = (float)JY901.stcAngle.Angle[1]/32768*180;

  //选择食指
  selectMuxPin(2);
  JY901.GetAngle();
  indexjy=(float)JY901.stcAngle.Angle[1]/32768*180;
  indexjz=(float)JY901.stcAngle.Angle[2]/32768*180+sum2;
  selectMuxPin(3);
  JY901.GetAngle();
  indexzy=(float)JY901.stcAngle.Angle[1]/32768*180;
  indexzz=(float)JY901.stcAngle.Angle[2]/32768*180;
  
  //选择中指
  selectMuxPin(5);
  selectMuxPin2(1);
  JY901.GetAngle();
  midjy=(float)JY901.stcAngle.Angle[1]/32768*180+sum3;
  midjz=(float)JY901.stcAngle.Angle[2]/32768*180;
  selectMuxPin(4);
  JY901.GetAngle();
  midzy=(float)JY901.stcAngle.Angle[1]/32768*180;
  midzz=(float)JY901.stcAngle.Angle[2]/32768*180;
  
  //选择无名指
  selectMuxPin(5);
  selectMuxPin2(2);
  JY901.GetAngle();
  ringjy=(float)JY901.stcAngle.Angle[1]/32768*180+sum4;
  ringjz=(float)JY901.stcAngle.Angle[2]/32768*180;
  selectMuxPin(5);
  selectMuxPin2(3);
  JY901.GetAngle();
  ringzy=(float)JY901.stcAngle.Angle[1]/32768*180;
  ringzz=(float)JY901.stcAngle.Angle[2]/32768*180;
  
  //选择小拇指
  selectMuxPin(5);
  selectMuxPin2(4);
  JY901.GetAngle();
  litjy=(float)JY901.stcAngle.Angle[1]/32768*180+sum5;
  litjz=(float)JY901.stcAngle.Angle[2]/32768*180;
  selectMuxPin(5);
  selectMuxPin2(5);
  JY901.GetAngle();
  litzy=(float)JY901.stcAngle.Angle[1]/32768*180;
  litzz=(float)JY901.stcAngle.Angle[2]/32768*180;
  //——————————————————————————————————————————
  //上面对传感器数据进行采集，后面对传感器数据转化为关节数据
  //先处理掌骨数据,模型中
  bpalmx = -palmx;bpalmy = -palmy;bpalmz = -palmz;
  
  //处理食指关节
  bindexjy = getJ(palmy,palmz,indexjy,indexjz).temp1;
  bindexjz = getJ(palmy,palmz,indexjy,indexjz).temp2;
  bindexzy = getM(indexjy,indexzy);
  bindexyy = getF(indexjy,indexzy);
  
  //处理中指关节
  bmidjy = getJ(palmy,palmz,midjy,midjz).temp1;
  bmidjz = getJ(palmy,palmz,midjy,midjz).temp2;
  bmidzy = getM(midjy,midzy);
  bmidyy = getF(midjy,midzy);

  //处理无名指关节
  bringjy = getJ(palmy,palmz,ringjy,ringjz).temp1;
  bringjz = getJ(palmy,palmz,ringjy,ringjz).temp2;
  bringzy = getM(ringjy,ringzy);
  bringyy = getF(ringjy,ringzy);

  //处理小拇指
  blitjy = getJ(palmy,palmz,litjy,litjz).temp1;
  blitjz = getJ(palmy,palmz,litjy,litjz).temp2;
  blitzy = getM(litjy,litzy);
  blityy = getF(litjy,litzy);

  //处理大拇指关节
  bbigjy = palmy - bigjy;
  bbigjz = palmz - bigjz;
  bbigzy = bigjz - bigzz;
  if(bbigzy<0){
      bbigzy = -bbigzy;
    }
   if(bbigzy>90){
      bbigzy = bbigzy - 90;
    }

  Serial.print(bpalmx);Serial.print(",");Serial.print(bpalmy);Serial.print(",");Serial.print(bpalmz);Serial.print(",");
  Serial.print(bindexjy);Serial.print(",");Serial.print(bindexjz);Serial.print(",");Serial.print(bindexzy);Serial.print(",");Serial.print(bindexyy);Serial.print(",");
  Serial.print(bmidjy);Serial.print(",");Serial.print(bmidjz);Serial.print(",");Serial.print(bmidzy);Serial.print(",");Serial.print(bmidyy);Serial.print(",");
  Serial.print(bringjy);Serial.print(",");Serial.print(bringjz);Serial.print(",");Serial.print(bringzy);Serial.print(",");Serial.print(bringyy);Serial.print(",");
  Serial.print(blitjy);Serial.print(",");Serial.print(blitjz);Serial.print(",");Serial.print(blitzy);Serial.print(",");Serial.print(blityy);Serial.print(",");
  Serial.print(bbigjy);Serial.print(",");
  Serial.print(bbigjz);Serial.print(",");
  Serial.println(bbigzy);
}
//-----------------------------------------------------------------------------------
//手掌校准模型
Data getJ(float zhangy,float zhangz,float datay,float dataz)
{
    Data temp;
    //下面对上下波动进行控制
    temp.temp1 = zhangy-datay;
    if(temp.temp1<0)temp.temp1 = - temp.temp1;
    if(temp.temp1>90)temp.temp1 = 90;
    //下面对左右晃动进行控制
    temp.temp2 = zhangz - dataz;
    if(temp.temp2<-10)temp.temp2 = -10;
    if(temp.temp2>10)temp.temp2 = 10;
    return temp;
}
//使用近指数据和中指传感器数据，获取模型中中指数据.PIP。jy表示近指传感器y轴数据，My表示中指传感器y轴数据
float getM(float jy,float My)
{
    float data = jy-My;
    if(data<0){
        data = -data;
      }
    if(data>110){
        data = 110;
      }
    return data;
}
//使用近指数据和中指弯曲数据，推出远指弯曲
float getF(float jy,float My)
{
    float data = jy-My;
    if(data<0){
        data = -data;
      }
    if(data>110){
        data = 110;
      } 
    data = 0.6777 *data;
    return data;
}
//-----------------------------------------------------------------------------------------
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
