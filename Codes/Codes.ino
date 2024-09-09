# include <SoftwareSerial.h>
# include "I2Cdev.h"
# include "MPU6050.h"
# include <MsTimer2.h>
# if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    # include "Wire.h"
# endif

/*******************引脚定义************************/
# define trigPin 14
# define echoPin 15     // 超声波测距模块
# define Buzzer 11       // 蜂鸣器
# define A_1A 6
# define A_1B 10   // R
# define B_1A 5
# define B_1B 9     // L  // 电机驱动模块
# define EncoderL_A 2
# define EncoderL_B 7
# define EncoderR_A 3
# define EncoderR_B 8   // N20减速电机编码器
// SCL -> SCL
// SDA -> SDA     //mpu6050

SoftwareSerial BleSerial (16, 17);   // RX, TX
MPU6050 mpu;

/***********************global variable***********************/
bool mpuState = false;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax_offset, ay_offset, az_offset;
float gx_offset, gy_offset, gz_offset;
float ax_new, ay_new, az_new, gx_new, gy_new, gz_new;
float ypr[2] = {0, 0};

float time_fliter;
float tau_fliter = 0.7;  // MPU6050互补滤波器时间常数 tau(sec)

String cmdd1, cmdd11, cmdd2, cmdd3;   // Remote control variable

float ppsR;
float ppsL;
float velocity_R;
float velocity_L;

float angle_balance_Kp = + 30, angle_balance_Kd = + 1;
float velocity_balance_Kp = - 0.3, velocity_balance_KI = velocity_balance_Kp / 200;
float Mechanical_balance = 0;     //机械中值(控制前进与后退）
float rightTurn = 0, leftTurn = 0;    //转弯附加值(控制转向）
float velocity_process = 0, velocity_Integral;
float Output_balance;

bool flag_Fall = false;
bool flag_Near = false;

int Flash_num = 0;

/******************Mpu6050初始化**********************/

void mpu6050Setup(){  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();   // 建立 Mpu6050 连接
  if(mpu.testConnection())  mpuState = true;  
  
  // 间隔0.05s采样10次，计算Acc与Gyro的Offset值
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for(int i = 0; i < 10; i++)       
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += float(ax);
    ay_sum += float(ay);
    az_sum += float(az);
    gx_sum += float(gx);
    gy_sum += float(gy);
    gz_sum += float(gz);
    delay(50);
  }
  ax_offset = ax_sum / 10;
  ay_offset = ay_sum / 10;
  az_offset = az_sum / 10 - 16384;
  gx_offset = gx_sum / 10;
  gy_offset = gy_sum / 10;
  gz_offset = gz_sum / 10; 

//  Serial.print("Offset_Acc:\tax:");
//  Serial.print(ax_offset);  Serial.print("\t");
//  Serial.print("ay:");
//  Serial.print(ay_offset);  Serial.print("\t");
//  Serial.print("az:");
//  Serial.println(az_offset);  
//  Serial.print("Offset_Gyro:\tgx:");
//  Serial.print(gx_offset);  Serial.print("\t");
//  Serial.print("gy:");
//  Serial.print(gy_offset);  Serial.print("\t");
//  Serial.print("gz:");
//  Serial.println(gz_offset);

//  ax_offset = 156.00;
//  ay_offset = -832.40;
//  az_offset = -33977.20;
//  gx_offset = -234.40;
//  gy_offset = -72.00;
//  gz_offset = -310.60; 
  
}

/*************Mpu6050互补滤波器 获取姿态 ********************/ 

void mpu6050Fliter(){
  if(!mpuState) return;
  float dt = (millis() - time_fliter) * 0.001;       // 滤波器运行周期（sec）
  time_fliter = millis();
  float a_fliter = tau_fliter / (tau_fliter + dt);  // 互补滤波器系数 
  float ypr_x_Acc, ypr_y_Acc;   
                          
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
  ax_new = float(ax) - ax_offset;    // 默认32768为 2g
  ay_new = float(ay) - ay_offset;
  az_new = float(az) - az_offset;   
  gx_new = - (float(gx) - gx_offset) / 32768 * 250;    // 默认32768为 250度/s
  gy_new = - (float(gy) - gy_offset) / 32768 * 250;     // 转化为角度制 °/s
  gz_new = - (float(gz) - gz_offset) / 32768 * 250;    

  ypr_x_Acc = atan2(ay_new, sqrt(ax_new * ax_new + az_new * az_new)) * 180/3.141592654;            // 由加速度计得出分别绕x,y旋转的倾角（无法得到绕z倾角）
  ypr_y_Acc = atan2(ax_new, sqrt(ay_new * ay_new + az_new * az_new)) * 180/3.141592654;             // 返回角度制 °
   
  ypr[0] = a_fliter * (ypr[0] + gx_new * dt) + (1 - a_fliter) * ypr_x_Acc;  /**/// 对陀螺仪得到倾角的低通滤波（时间太长有温漂）
  ypr[1] = a_fliter * (ypr[1] + gy_new * dt) + (1 - a_fliter) * ypr_y_Acc;       // 并且对加速度计得到倾角的高通滤波（时间太短有干扰）

}

/*********************减速电机编码器测转速*****************/

void IntEncoder_R(){
  if(digitalRead(EncoderR_B) == LOW)
  ppsR ++;
  else if(digitalRead(EncoderR_B) == HIGH) 
  ppsR --;
}
void IntEncoder_L(){
  if(digitalRead(EncoderL_B) == LOW)  //两电机位置刚好对称相反
  ppsL --;
  else if(digitalRead(EncoderL_B) == HIGH) 
  ppsL ++;
}

void enCounter(){
  velocity_R = 360 * ppsR / 35;  // 转速为 360° * pps/(7*0.1*50)
  velocity_L = 360 * ppsL / 35;   // 角度制 °/s
  ppsR = 0;
  ppsL = 0;
}

/***************超声波测距判断障碍物*****************/
void isDistance(){
  float duration_us,distance_cm;
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin,LOW);
  duration_us = pulseIn(echoPin, HIGH);
  distance_cm = duration_us * 0.017;
  if(distance_cm <= 30 && distance_cm > 5) flag_Near = true;
    else flag_Near = false;
    Serial.println(distance_cm);
}

/**********定时中断函数, 同时执行多项任务****************/
void flash(){
  Flash_num ++;
  
  if(Flash_num % 100 == 0)  enCounter();  // 每0.1s执行一次 函数enCounter()
  
  // if(Flash_num % 4000 == 0) isDistance(); // 每4s执行一次 函数isDistance()
}

/*********************角度环 PD*****************/
float angleBalance(float ypr[], float gx_new){
  float Bias = ypr[0] - Mechanical_balance;
  float Angle_balance = angle_balance_Kp * Bias + angle_balance_Kd * gx_new;
  Serial.print("angle_balance_Bias:\t");
  Serial.println(Bias);
  if(Bias >= 30 || Bias <= -30)   flag_Fall = true;       // 判断是否倒下
    else  flag_Fall = false;
  
  return (Angle_balance);
}

/*********************速度环 PI*******************/
float velocityBalance(float velocity_R, float velocity_L){
  float velocity_least;
  float velocity_balance;

  if(flag_Fall) 
  {
    velocity_process = 0;
    velocity_Integral = 0;
  }
    else
    {
      velocity_least = (velocity_R + velocity_L) - 0;
      
      velocity_process *= 0.7; // 一阶低通滤波器减小对角度环影响
      velocity_process += velocity_least * 0.3;
    
      velocity_Integral += velocity_process;       //积分出角位移，积分时间：0.1s
      if(velocity_Integral > 10000) velocity_Integral = 10000;      //积分上下限
      if(velocity_Integral < -10000) velocity_Integral = -10000;
    } 
  velocity_balance = velocity_process * velocity_balance_Kp + velocity_Integral * velocity_balance_KI;
   
  return velocity_balance;
}

/****************直立平衡 PWM******************************
 * 角度环与速度环串级输出
 */
void selfBalance(float ypr[], float gx_new, float velocity_R, float velocity_L){
  Output_balance = angleBalance(ypr, gx_new) + velocityBalance(velocity_R, velocity_L);
  
  if(flag_Fall) Output_balance = 0;

  if(Output_balance<0) 
  {
    if(Output_balance<-255) 
    { 
      analogWrite(A_1B, 0);
      analogWrite(A_1A, 255 - leftTurn);
      analogWrite(B_1A, 255 - rightTurn);
      analogWrite(B_1B, 0);
    }
    else 
    {
      analogWrite(A_1B, 0);
      analogWrite(A_1A, - Output_balance + rightTurn);
      analogWrite(B_1A, - Output_balance + leftTurn);
      analogWrite(B_1B, 0);
    }    
  }
  else 
  {
    if(Output_balance>255) 
    { 
      analogWrite(A_1B, 255 - rightTurn);
      analogWrite(A_1A, 0);
      analogWrite(B_1A, 0);
      analogWrite(B_1B, 255 - leftTurn);
    }
    else 
    {
      analogWrite(A_1B, Output_balance + leftTurn);
      analogWrite(A_1A, 0);
      analogWrite(B_1A, 0);
      analogWrite(B_1B, Output_balance + rightTurn);
    }
  } 
}

/******************远程操控*********************/

void appControl(){
  char cmd;
  String cmdd = "";
  
//  BleSerial.println();
//  BleSerial.println(int(ypr[0]));   // 手机端查看小车运动数据（车身倾角,轮子转速...）
//  BleSerial.print(" L");
//  BleSerial.print(int(velocity_L));
//  BleSerial.print(" R");
//  BleSerial.print(int(velocity_R));
 
  while(BleSerial.available() && ((cmd = BleSerial.read()) != '\0'))
  { 
    cmdd += cmd;
    delay(10);
  }
  
  if(cmdd == "A" || cmdd == "B" || cmdd == "C" || cmdd == "D" ||cmdd == "E") cmdd1 = cmdd;   //控制小车
  else if(cmdd == "G" || cmdd == "H") cmdd2 = cmdd;   //开闭蜂鸣器
  else if(cmdd == "P" || cmdd == "Q" || cmdd == "R" || cmdd == "S") cmdd3 = cmdd;  //调节PID参数
  //  else if(...)  ...;

  if(flag_Fall) cmdd1 = "E";
  else if(flag_Near) cmdd1 = "C";  
  
  // 远程操控小车
  if(cmdd1 == "A") {           
    if(rightTurn > 0)  rightTurn -= 0.5; 
    if(leftTurn > 0) leftTurn -= 0.5;   
    if(Mechanical_balance >= - 10) Mechanical_balance -= 0.04;
    if(rightTurn != 0 || leftTurn != 0 || Mechanical_balance >= - 10)  delay(5);
  }
  
  else if(cmdd1 == "B") {
    if(rightTurn > 0)  rightTurn -= 0.5; 
    if(leftTurn > 0) leftTurn -= 0.5;   
    if(Mechanical_balance <= 10) Mechanical_balance += 0.04;
    if(rightTurn != 0 || leftTurn != 0 || Mechanical_balance <= 10)  delay(5);
  }
  
  else if(cmdd1 == "C") {
    if(rightTurn > 0)  rightTurn -= 0.5;
    if(leftTurn <= 65)  leftTurn += 0.5;
    if(Mechanical_balance < 0)  Mechanical_balance += 0.04;
      else if(Mechanical_balance > 0)  Mechanical_balance -= 0.04;   
    if(rightTurn != 0 || leftTurn <= 65 || Mechanical_balance != 0)  delay(5);   
  }
  
  else if(cmdd1 == "D") {
    if(rightTurn <= 70) rightTurn += 0.5;
    if(leftTurn > 0)  leftTurn -= 0.5;
    if(Mechanical_balance < 0)  Mechanical_balance += 0.04;
      else if(Mechanical_balance > 0)  Mechanical_balance -= 0.04;   
    if(rightTurn <= 70 || leftTurn != 0 || Mechanical_balance != 0)  delay(5);
  }
  
  else if(cmdd1 = "E"){
    if(rightTurn > 0)  rightTurn -= 0.5;
    if(leftTurn > 0)  leftTurn -= 0.5;
    if(Mechanical_balance < 0)  Mechanical_balance += 0.04;
      else if(Mechanical_balance > 0)  Mechanical_balance -= 0.04; 
    if(rightTurn != 0 || leftTurn != 0 || Mechanical_balance != 0)  delay(5);
  }

  // 开闭蜂鸣器
  if(cmdd2 == "G") {
    digitalWrite(Buzzer, HIGH);
  }
  else if(cmdd2 == "H") {
    digitalWrite(Buzzer, LOW);
  }

  // 调节PD /PI参数
  if(cmdd3 == "P") {
    cmdd3 = "";
    while(BleSerial.available() && (cmd = BleSerial.read()) != '\0'){
      cmdd3 += cmd;  
      delay(10);          
    }
    angle_balance_Kp = + atoi(cmdd3.c_str()) / 100;                            
  }
  
  else if(cmdd3 == "Q") {
    cmdd3 = "";
    while(BleSerial.available() && (cmd = BleSerial.read()) != '\0'){
      cmdd3 += cmd;
      delay(10);            
    }
    angle_balance_Kd = + atoi(cmdd3.c_str()) / 100;   
  }   
  
  else if(cmdd3 == "R") {
    cmdd3 = "";
    while(BleSerial.available() && (cmd = BleSerial.read()) != '\0'){
      cmdd3 += cmd;
      delay(10);            
    }
    velocity_balance_Kp = - atoi(cmdd3.c_str()) / 100;
    velocity_balance_KI = velocity_balance_Kp / 200;   
  } 
  
  else if(cmdd3 == "S") { 
    angle_balance_Kp = + 30;
    angle_balance_Kd = + 1;
    velocity_balance_Kp = - 0.3;
    velocity_balance_KI = velocity_balance_Kp / 200;
  }
}   

void setup()
{
  Serial.begin(57600);
  while(!Serial) ;
  BleSerial.begin(9600);
  
  attachInterrupt(INT0, IntEncoder_L, FALLING);
  attachInterrupt(INT1, IntEncoder_R, FALLING); 
  MsTimer2::set(1, flash);    // 中断设置函数，每1ms执行一次 
  MsTimer2::start(); // 开始计时
  pinMode(EncoderR_B, INPUT);
  pinMode(EncoderL_B, INPUT); // 检测编码器的正负
  
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);

  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  
  pinMode(A_1A, OUTPUT);
  pinMode(A_1B, OUTPUT);                              
  pinMode(B_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  digitalWrite(SCL, 0);
  digitalWrite(SDA, 0);
  
  mpu6050Setup();
}

void loop()
{  
  mpu6050Fliter();
  selfBalance(ypr, gx_new, velocity_L, velocity_R);
  appControl();
}
