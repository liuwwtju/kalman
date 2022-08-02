/******加速度计无法计算偏航角，陀螺仪计算偏航有漂移，需与磁力计融合******/
#include <Wire.h>
#include <Kalman.h>
#define RESTRICT_PITCH
#define fRad2Deg  57.295779513f //将弧度转为角度的乘数
#define fDeg2Rad  0.0174532925f
#define MPU 0x68 //MPU-6050的I2C地址
#define nValCnt 7 //一次读取寄存器的数量

float kalAngleX, kalAngleY; //横滚、俯仰、偏航角的卡尔曼融合值
Kalman kalmanX; // 实例化卡尔曼滤波
Kalman kalmanY;
//Kalman kalmanZ;
long timer;

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050
  //WriteMPUReg(0x6A, ReadMPUReg(0x6A)&0xDF);
  WriteMPUReg(0x37, ReadMPUReg(0x37)|0x02);  //开启mpu6050的IIC直通，连接磁场传感器
  float realVals[nValCnt];
  for(int i=0;i<500;i++)ReadAccGyr(realVals); //读出测量值
  float roll,pitch;
  GetRollPitch(realVals,&roll,&pitch);
  roll *= fRad2Deg; pitch *= fRad2Deg;
  kalmanX.setAngle(roll); // 设置初始角
  kalmanY.setAngle(pitch);
  
  timer = micros(); //计时
}

void loop() {
  float realVals[nValCnt];
  ReadAccGyr(realVals); //读出测量值
  
  double dt = (double)(micros() - timer) / 1000000; // 计算时间差
  timer = micros();  //更新时间
  
  float roll,pitch;
  GetRollPitch(realVals,&roll,&pitch);

  float gyroXrate = realVals[4] / 131.0; // 转换到角度/秒
  float gyroYrate = realVals[5] / 131.0;
  // 解决加速度计角度在-180度和180度之间跳跃时的过渡问题
  roll *= fRad2Deg; pitch *= fRad2Deg; 
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } 
  else{
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // 卡尔曼融合
  }
  if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate; // 限制的加速度计读数
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  //对俯仰角滤波
  
  Serial.print("accangleX:");Serial.print(roll);
  Serial.print(" kalAngleX:");Serial.print(kalAngleX);
  Serial.print(" accangleY:");Serial.print(pitch);
  Serial.print(" kalAngleY:");Serial.print(kalAngleY);
  Serial.print("\r\n");
  delay(100);
}


//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(float *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (int i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//算得Roll角。
void GetRollPitch(float *pRealVals,float* roll,float* pitch) {
#ifdef RESTRICT_PITCH
  float fNorm = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  *pitch = atan2(-pRealVals[0],fNorm);
  *roll = atan2(pRealVals[1],pRealVals[2]);  //atan2和atan作用相同，但atan2在除数是0时也可以计算，所以尽量使用atan2
#else
  float fNorm = sqrt(pRealVals[2] * pRealVals[2] + pRealVals[0] * pRealVals[0]);
  *roll = atan2(pRealVals[1],fNorm);
  *pitch = atan2(-pRealVals[0],pRealVals[2]);
#endif
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}
