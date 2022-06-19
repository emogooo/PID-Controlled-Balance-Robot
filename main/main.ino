//kütüphaneleri çağırıyoruz.
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 20 //motorları sürerken kullanılacak hız değişkeni

MPU6050 mpu; //bir MPU6050 nesnesi yaratıyoruz.

// MPU kontrolü için değişkenleri tanımlıyoruz
bool dmpReady = false; // DMP başlatma başarılı ise TRUE olacak
uint8_t mpuIntStatus; // interrupt status byte bilgilerini tutacak
uint8_t devStatus; // cihazın çalışma durumu bilgisi (0 = başarılı, !0 = HATA)
uint16_t packetSize; // DMP paket boyutu (standardı 42 byte)
uint16_t fifoCount; // FIFO stackteki tüm byteları sayıyor
uint8_t fifoBuffer[64]; // FIFO hafıza kapasitesi

// pozisyon/hareket değişkenleri
Quaternion q; // [w, x, y, z] Dördeyleri saklayacak
VectorFloat gravity; // [x, y, z] yerçekimi vektörü
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll eksen bilgisi

//PID değişkenleri
// setpoint kaç derecede düz durduğu. Sizin ki 1-2 derece farklı olabilir.
double originalSetpoint = 184;   //175
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

// bu sayılar her farklı robot için değişir siz de kendi katsayılarınızı deneme yanılma ile belirleyin
float Kp = 0;   //40
float Kd = 0;  //1.3
float Ki = 0;   //160
float toplam = 0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6; //sağ ve sol motorun hangi hızla döneceği katsayıları aralarında //0,63
//eşitsizlik olabilir.
double motorSpeedFactorRight = 0.65;    //0,60
//MOTOR ÇIKIŞLARI
int ENA = 3;   //5
int IN1 = 4;   //6
int IN2 = 5;    //5
int IN3 = 9;    //8
int IN4 = 8;    //9
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // MPU interrrupt pini HIGH olmuş mu ona bakıyor
void dmpDataReady()
{
  mpuInterrupt = true;
}

//////////////////////////POT PID AYAR///////////////////////////////

float MKP = 0.5;
float MKI = 1;
float MKD = 0.03125;

float lastP = 0;
float lastI = 0;
float lastD = 0;

bool flag;

bool e_pid_ayar(){ 
  /*Kp = analogRead(A0)*MKP;
  Ki = analogRead(A1)*MKI;
  Kd = analogRead(A2)*MKD;
  if(abs(Kp+Ki+Kd - toplam) < 0.5){
    return false;
  }else{
    toplam = Kp+Ki+Kd;*/
    Kp = analogRead(A0)*MKP;
    Ki = analogRead(A1)*MKI;
    Kd = analogRead(A2)*MKD;
    
    
    Serial.print("Kp: ");Serial.print(Kp);
    Serial.print(" Ki: ");Serial.print(Ki);
    Serial.print(" Kd: ");Serial.println(Kd);  
    return true;
  
}

bool pid_ayar(){ 
  // 1 / 1024 ölü bölge
  flag = false;
  Kp = analogRead(A0) * MKP;
  Ki = analogRead(A1) * MKI;
  Kd = analogRead(A2) *MKD;

  if(abs(lastP - Kp) > MKP || abs(lastI - Ki) > MKI || abs(lastD - Kd) > MKD){
    flag = true;
  }
  
  if(abs(lastP - Kp) <= MKP){
    Kp = lastP;
  }
  if(abs(lastI - Ki) <= MKI){
    Ki = lastI;
  }
  if(abs(lastD - Kd) <= MKD){
    Kd = lastD;
  }

  if(flag){
    lastP = Kp;
    lastI = Ki;
    lastD = Kd;
    Serial.print("Kp: ");Serial.print(Kp);
    Serial.print(" Ki: ");Serial.print(Ki);
    Serial.print(" Kd: ");Serial.println(Kd);  
    return true;
  }
  else{
    return false;
  }
}

void setup()
{
  Serial.begin(115200);
  // I2C Bus'a bağalanıyoruz (I2Cdev kütüphanesi bunu otomatik yapmaz)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C saat hızı 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  //Serial.println("IMU başlatılıyor");
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // hassasiyet için buraya yukarıdaki anlatılan yöntemle aldığınız kendi gyro offset değerlerinizi girebilirsiniz.
  mpu.setXGyroOffset(-80);
  mpu.setYGyroOffset(-23);
  mpu.setZGyroOffset(12);
  mpu.setZAccelOffset(1896); 
  // cihaz çalıştı mı kontrol ediyoruz(başarılı ise 0 döndürecek)
  if (devStatus == 0)
  {
    // DMP açılıyor
    mpu.setDMPEnabled(true);
    // Arduino interrupt tespiti etkinleştiriliyor
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // main loop kullansın diye DMP Hazır değişkenine true atıyoruz
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    //PID ayarlanıyor
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10); //örnekleme zamanı  //10
    pid.SetOutputLimits(-255, 255); //çünkü motorların limiti bu
  }
  else
  {
    // HATA!
    // 1 = Ön Hafıza Yükleme Başarısız!
    // 2 = DMP konfigürasyon güncellemesi hatalı!
    // (Genelde 1 numaralı hata oluşabilir)
    Serial.print(F("DMP Başlatma Hatalı (Hata Kodu "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


void loop()
{
  // Eğer hata varsa boşuna uğraşma çık
  if (!dmpReady) return;
  
  // MPU interrupt için veya ilave paketler için bekle
  
  while (!mpuInterrupt && fifoCount < packetSize)
  { 
    //PID hesapları yapılıyor ve motorlar sürülüyor 
    if(pid_ayar()){
      pid.SetTunings(Kp, Ki, Kd);    
    }
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
    
  }
  
  // Interrupt bayrağı resetleniyor ve INT_STATUS_BYTE verisi alınıyor
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // O anki FIFO sayısını alıyor
  fifoCount = mpu.getFIFOCount();
  
  // FIFO stack'de overflow hatası var mı ona bakıyor.
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // FIFO resetleniyor ki overflow temizlensin
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
    
    // aksi taktirde DMP interrupt var mı ona bakıyor (sıklıkla olan bir durum)
  }
  else if (mpuIntStatus & 0x02)
  {
    // doğru veri uzunluğu için kısa bir bekleme
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // FIFO stack'ten veri okunuyor.
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // 1 ve daha fazla paket varsa burada FIFO içeriği sayılıyor böylece interrupt olmadan hemen okuma sağlanıyor
    
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180;
  }

}
