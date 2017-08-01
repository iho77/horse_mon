#define MY_RADIO_NRF24
#define MY_NODE_ID 2
#define SER_DEBUG 1
#define OUTPUT_SERIAL 1
#define MY_RADIO_NRF24
#define MY_NODE_ID 2



#include <MyConfig.h>
#include <MySensors.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <MsTimer2.h>
#include <Time.h>
#include <TimeLib.h>
#include <TroykaIMU.h>
#include <Math.h>



#define TEMP_SENSOR_PIN 0
#define VLTG_PIN 1
#define VLT_PER_DGR 0.02

#define TIMER_PERIOD 100
#define MIN_PERIOD 60*(1000/TIMER_PERIOD)
#define SEC_PERIOD 1000/TIMER_PERIOD
#define IMU_PERIOD 1


#define HEAD_DOWN_THR -10 
#define HEAD_UP_THR +10


// множитель фильтра
#define BETA 0.22f



class mGyro:public Gyroscope {
  public:
  void lowpower(bool enable); 
};

void mGyro::lowpower(bool enable){
  if (enable)
        _ctrlReg1 &= B11110111;
    else
       _ctrlReg1 |= (1 << 3);

    writeCtrlReg1();
}

class mAccel:public Accelerometer {
  public:
  void lowpower(); 
};


void mAccel::lowpower(){
  _ctrlReg1 = B00101111;
     writeCtrlReg1();
}

class mPeak{
 public:
  mPeak();
  int GetBin(int num);
  void Push(float value);
  int GetCount();
  void Reset();
  void SetThr(float a, float b, float c);
 private:
    float _data[3];
    int _bin[3];
    float _thr[3];
}; 

mPeak::mPeak(){
  
  for (int x = 0; x<3; x++){
      _data[x] = 0.0;
      _bin[x] = 0;
      _thr[x] = 0.0;
  }
  
}

void mPeak::Push(float value){
  _data[2]=_data[1];
  _data[1]=_data[0];
  _data[0]=value;
  
  if ((_data[1] > _data[2]) && (_data[1] > _data[0])) {
     if (_data[1] > _thr[0]) ++_bin[2]; else if ( (_data[1] <= _thr[0]) && (_data[1] > _thr[1]) )  ++_bin[1]; else if ( (_data[1] <= _thr[1]) && (_data[1] > _thr[2]) ) ++_bin[0];
     
    }



}

int mPeak::GetCount(){
  return _bin[0]+_bin[1]+_bin[2];
}

void mPeak::Reset(){
  _bin[0] = 0;
  _bin[1] = 0;
  _bin[2] = 0;
}

int mPeak::GetBin(int num){
  int x = _bin[num];
  return x;
}
 void mPeak::SetThr(float a,float b,float c){
  _thr[0] = a;
  _thr[1] = b;
  _thr[2] = c;
 }

 
// создаём объект для фильтра Madgwick
Madgwick filter;
 
// создаём объект для работы с акселерометром
mAccel accel;
// создаём объект для работы с гироскопом
mGyro gyro;

//Barometer baro;
 
// переменные для данных с гироскопов, акселерометров
volatile float gx, gy, gz, ax, ay, az;
 
// получаемые углы ориентации
float yaw, pitch, roll;
 
// переменная для хранения частоты выборок фильтра
float fps = 1000 / TIMER_PERIOD;

volatile float curTemp = 0.0;
volatile float curVolt = 0.0;
volatile float curAltitude;
volatile float prevAltitude;
volatile int altProbeCount;
volatile boolean lightIsOn;
volatile boolean manualMode;


float last_mag[2];
float last_y[2];

float sensors[11];

// sensors[0] - temp
#define S_TEMP 0
// sensors[1] - volt
#define S_VOLT 1
// sensors[2] - light movements
#define S_LM 2
//sensors[3] - medium movements
#define S_MM 3
// sensors[4] - power movementss 
#define S_PM 4
//sensors[5] - light pitching
#define S_LR 5
// sensors[6] - medium pitching
#define S_MR 6
// sensors[7] - power pitching
#define S_PR 7
// sensors[8] - secs head down
#define S_HD 8
// sensors[9] - secs head straight
#define S_HS 9
// sensors[10] - secs head up
#define S_HU 10
 

 



volatile int minCounter, secCounter, imuCounter;

mPeak mag, pr;

MyMessage msg(S_CUSTOM, V_VAR1);





void setup() {
  wdt_reset();
  int i; 
  float alt = 0;
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Loading...");

  accel.begin();
  accel.lowpower();
 
  gyro.begin();

  //baro.begin();
  

//  Serial.println("Calibrating altitude");
//  for (i=1; i<=100; i++){
//     alt += baro.GOST4401_altitude(baro.readPressurePascals());
//     delay(100);
//  }
//  Serial.println("Done");
 // prevAltitude = alt/i;
//  curAltitude = prevAltitude;
 // altProbeCount = 1;
 // Serial.print("Base alt:");
 // Serial.println(prevAltitude);
//  Serial.end();

 for (i=0;i<11;i++){
  sensors[i]=0;
 }
 
  
  minCounter = 0;
  secCounter = 0;
  imuCounter = 0;
  
  last_mag[0]=0;
  last_mag[1]=0;
  last_mag[2]=0;

  last_y[0]=0;
  last_y[1]=0;
  last_y[2]=0;

  mag.SetThr(0.5,0.05,0.005);
  pr.SetThr(30.0,10.0,4.0);
  
  wdt_enable(WDTO_8S);
  MsTimer2::set(TIMER_PERIOD, timerInterupt);
  MsTimer2::start();

}


void  timerInterupt() {

 wdt_reset();
 minCounter++;
 secCounter++;
 imuCounter++;
}

void loop() {

 
float temp, volt,modroll, altdelta;
int a,b,c;
int r1,r2,r3, i;

   

if (imuCounter >= IMU_PERIOD){

   imuCounter = 0;
   processIMU();
  // processBaro();
 
}


if ( secCounter >= SEC_PERIOD )  {

  curTemp += getTemperature();
  curVolt += getVoltage();
  
  float alt = curAltitude / altProbeCount;
  altProbeCount = 1;
  altdelta = alt - prevAltitude;
  prevAltitude = alt;
  curAltitude = alt;
  secCounter = 0;
  yaw =  filter.getYawDeg();
  pitch = filter.getPitchDeg();
  roll = filter.getRollDeg();

  modroll=sqrt(pow(roll,2));
  
  pr.Push(modroll);
  
  if (pitch > HEAD_UP_THR) {sensors[S_HU] ++;} else  {if (pitch < HEAD_DOWN_THR) {sensors[S_HD] ++;} else sensors[S_HS] ++;}

  if (SER_DEBUG) {
  Serial.begin(115200);
  Serial.print("yaw: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print("roll: ");
  Serial.println(roll);
  //Serial.print(" ");
 // Serial.print(" alt:");
 // Serial.println(altdelta);
  Serial.end();}
  
}


if ( minCounter >= MIN_PERIOD )  
{

 
 sensors[S_TEMP] = curTemp / 60.0;  //поменять если изменится период чтений температуры
 sensors[S_VOLT] = curVolt / 60.0;  //поменять если изменится период чтений напряжения
 curTemp = 0.0;
 curVolt = 0.0;

 sensors[S_LM] = mag.GetBin(0);
 sensors[S_MM] = mag.GetBin(1);
 sensors[S_PM] = mag.GetBin(2);
 mag.Reset();
 sensors[S_LR] = pr.GetBin(0);
 sensors[S_MR] = pr.GetBin(1);
 sensors[S_PR] = pr.GetBin(2);
 pr.Reset();

 if (OUTPUT_SERIAL){
  
 Serial.begin(115200);
 for (i=0;i<11;i++){ 
 Serial.print(" : ");
 Serial.print(sensors[i]);
 }
 Serial.println();
 Serial.end();} else {

  for (i=1; i<= 11; i++){
  // msg.setSensor(i);
  // msg.set(dht.getHumidity());
  // send(msg);
  }
  
 }

 for (i=8;i<11;i++) sensors[i]=0;
/*
msg.setSensor(1);
msg.set(dht.getTemperatureC());
send(msg);
 
msg.setSensor(2);
msg.set(dht.getHumidity());
send(msg);

msg.setSensor(3);
msg.set(lightIsOn);
send(msg);
 
msg.setSensor(4);
msg.set(manualMode);
send(msg);

*/
minCounter=0;



}

sleepNow();
}

void sleepNow()
{

     
    // Choose our preferred sleep mode:
    RF24_powerDown();
    gyro.lowpower(true);
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    power_adc_disable();
    power_twi_disable();
    power_spi_disable();


 
    // Set sleep enable (SE) bit:
    sleep_enable();
    sleep_bod_disable();
 
    // Put the device to sleep:
    sleep_mode();
 
    // Upon waking up, sketch continues from this point.
    sleep_disable();
    power_all_enable();
    gyro.lowpower(false);
}

void processIMU(){
 
  float magnitude;
 
    // считываем данные с акселерометра в единицах G
 
  accel.readGXYZ(&ax, &ay, &az);

  magnitude = sqrt(pow(ax,2)+pow(ay,2)+pow(az,2))-1;
  
  if (magnitude<0.001) magnitude = 0;

  mag.Push(magnitude);
  
// считываем данные с акселерометра в радианах в секунду

  gyro.readRadPerSecXYZ(&gx, &gy, &gz);


 
  
// устанавливаем коэффициенты фильтра
 
   filter.setKoeff(fps, BETA);
// обновляем входные данные в фильтр
 
   filter.update(gx, gy, gz, ax, ay, az);

  
}


void processBaro(){
  float alt, delta;
 // alt = baro.GOST4401_altitude(baro.readPressurePascals());
  altProbeCount ++;
  delta = prevAltitude - alt ;
  if (delta < 0) {delta = delta * (-1.0);}
  if (delta > 2) { curAltitude += prevAltitude;  } else {  curAltitude += alt;  }
  
}

float getTemperature()   // Чтение показаний датчика температуры
{
  int sensorvalue=analogRead(TEMP_SENSOR_PIN);
  return sensorvalue*5.0/1024.0 / VLT_PER_DGR;
}

float getVoltage()   // Чтение показаний датчика температуры
{
  int sensorvalue=analogRead(VLTG_PIN);
  return sensorvalue*5.0/1024.0 *2;
}



/*

void receive(const MyMessage &message)
{

  if (message.sensor == 3 && message.type == 3) {
     manualMode = false;
  }
  if (message.sensor == 3 && message.type == 2) {
     manualMode = true;
  }
  
  if (message.sensor == 3 && message.type == 1 && manualMode) {
  
    Serial.println("Got message switch on");
    lightIsOn=true;
    digitalWrite(RELAY_PIN, HIGH); 
 
  }
  if (message.sensor == 3 && message.type == 0 && manualMode) {
  
    Serial.println("Got message switch off");
    lightIsOn=false;
    digitalWrite(RELAY_PIN, LOW); 
 
  }
}


void checkOnOff(){

  if (manualMode == false) {
  RTC.read(tm);  
  Serial.print("check time:");
  Serial.println(tm.Hour
  );
  if ((tm.Hour >= onHour) && (tm.Hour < offHour)){
    if(lightIsOn == false){
    Serial.println("switch on");  
    lightIsOn = true;
    digitalWrite(RELAY_PIN, HIGH); 
   }}
    else {
    if(lightIsOn){
    Serial.println("switch off");  
    lightIsOn = false ;
    digitalWrite(RELAY_PIN, LOW); 
   }}} 
  
}


*/





