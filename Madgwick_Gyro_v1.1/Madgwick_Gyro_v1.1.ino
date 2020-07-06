//================================================================//
//  BMX055から得たデータをMadgwickFilterに通して角度を得るプログラム           //
//  地磁気が悪影響を与える場合はFilter関数内の*1を有効化,*2をコメントアウト    //
//  角度は±90°、サンプリング周波数は挙動を見て調整(制御周期が遅いなら周波数下げ)//
//  KalmanFilterより計算負荷は小さいものの、収束は目に見えてわかる程度に遅い   //
//  Created by K.Takamiya(2020/06/26)                             //
//================================================================//
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <math.h>
Madgwick MadgwickFilter;
// BMX055　加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)
// 制御周期
#define T 50
/*3軸加速度データ格納*/
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
/*3軸角速度データ格納*/
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
/*地磁気データ格納*/
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;
/*MagdwickFilter通過後*/
float roll = 0;
float pitch = 0;
float yaw = 0;
/*積分関連(補正比較用)*/
float prexGyro = 0.00;
float preyGyro = 0.00;
float prezGyro = 0.00;
float xDeg = 0.00;
float yDeg = 0.00;
float zDeg = 0.00;
int dt = 0;
/*Loop回数格納用*/
long tick = 0;

//=====================================================================================//
void BMX055_Init(){
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl(){
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-2g
  yAccl = yAccl * 0.0098; // renge +-2g
  zAccl = zAccl * 0.0098; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro(){
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag(){
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] <<8) | (data[0]>>3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] <<8) | (data[2]>>3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] <<8) | (data[4]>>3));
  if (zMag > 16383)  zMag -= 32768;
}
//=====================================================================================//
void setup(){
  Wire.begin();             //I2CStart
  Serial.begin(9600);       //baudrate9600
  BMX055_Init();            //BMX055Initialize
  delay(300);
  MadgwickFilter.begin(10); //MadgwickFilter:Sampling rate 10Hz

}
//=====================================================================================//
void loop(){

  readSensor(); //Get sensor data
  Filter();     //MadgwickFilter
  //Integral();   //Intergral

  tick++;
  if(tick % 20 == 0){ //20s-1deaw
    tick = 0;
    draw_madg();
  //  draw_intg();
  }
}
//=====================================================================================//
void draw_madg(){
  Serial.println("==============================");
  Serial.print("Calibrated Angle X:");
  Serial.print(roll);
  Serial.print("\t||\t");
  Serial.print("Calibrated Angle Y:");
  Serial.print(pitch);
  Serial.print("\t||\t");
  Serial.print("Calibrated Angle Z:");
  Serial.println(yaw);
}

void draw_intg(){
  Serial.println("==============================");
  Serial.print("Integral Angle X:");
  Serial.print(xDeg);
  Serial.print("\t||\t");
  Serial.print("Integral Angle Y:");
  Serial.print(yDeg);
  Serial.print("\t||\t");
  Serial.print("Integral Angle Z:");
  Serial.println(zDeg);
}
//=====================================================================================//
void readSensor(){
  BMX055_Accl();//BMX055 Acceleration Data
  BMX055_Gyro();//BMX055 Gyro Data
  BMX055_Mag(); //BMX055 Magnetic Data
}
//=====================================================================================//
void Filter(){
  //MadgwickFilter.updateIMU(xGyro,yGyro,zGryo,xAccl,yAccl,zAccl);            //*1:地磁気除外用
  MadgwickFilter.update(xGyro,yGyro,zGyro,xAccl,yAccl,zAccl,xMag,yMag,zMag);  //*2:地磁気加味
  roll  = MadgwickFilter.getRoll();
  pitch = MadgwickFilter.getPitch();
  yaw   = MadgwickFilter.getYaw(); 
}
//=====================================================================================//

void Integral(){
  dt = T;
  /*X Axis*/
  xDeg += (prexGyro + xGyro)*dt/2;
  prexGyro = xGyro;
  /*Y Axis*/
  yDeg += (preyGyro + yGyro)*dt/2;
  preyGyro = yGyro;
  /*Z Axis*/
  zDeg += (prezGyro + zGyro)*dt/2;
  prezGyro = zGyro;
}
