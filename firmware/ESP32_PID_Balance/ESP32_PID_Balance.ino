#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_arduino_version.h>
#include "control_core.h"
#include "dashboard_html.h"

// PHYSICAL PWM OUTPUT: roll/pitch tuning prototype, not autonomous flight firmware.
const int MOTOR_PINS[4]={25,26,27,14}; // FR, RR, FL, RL
const uint8_t MPU=0x68;
const uint32_t SAMPLE_US=10000, HEARTBEAT_MS=1500;
const float TILT_LIMIT=35.0f;
WebServer server(80);
Adafruit_SSD1306 oled(128,64,&Wire,-1,400000,400000);
balance::AxisPID rollPID,pitchPID;
bool pwmOK=true,oledOK=false,imuOK=false,calibrating=false,calibrated=false,armed=false;
String fault="";
int throttle=1000,trims[4]={0,0,0,0},motors[4]={1000,1000,1000,1000};
float roll=0,pitch=0,rollRate=0,pitchRate=0,filteredRoll=0,filteredPitch=0;
float rollZero=0,pitchZero=0,gxBias=0,gyBias=0,ax=0,ay=0,az=0,tempC=0;
float correctionR=0,correctionP=0;
uint32_t bootMs=0,lastSample=0,lastGoodMs=0,lastBeat=0,generation=0,lastOLED=0,lastLog=0;
int calibrationCount=0;
double sumR=0,sumP=0,sumGX=0,sumGY=0;
float previousCalR=0,previousCalP=0;

void writeMotors(){
 for(int i=0;i<4;i++){
  uint32_t duty=((uint32_t)motors[i]*65535UL+10000UL)/20000UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[i],duty);
#else
  ledcWrite(i,duty);
#endif
 }
}
void stopOutputs(){
 armed=false;throttle=1000;generation++;
 for(int i=0;i<4;i++){trims[i]=0;motors[i]=1000;}
 correctionR=correctionP=0;rollPID.reset();pitchPID.reset();writeMotors();
}
void fail(const char *reason){stopOutputs();calibrated=false;calibrating=false;fault=reason;Serial.println(fault);}
bool readBytes(uint8_t reg,uint8_t *data,uint8_t count){
 Wire.beginTransmission(MPU);Wire.write(reg);
 if(Wire.endTransmission(false)!=0)return false;
 if(Wire.requestFrom(MPU,count,true)!=count)return false;
 for(int i=0;i<count;i++)data[i]=Wire.read();return true;
}
bool writeReg(uint8_t reg,uint8_t value){Wire.beginTransmission(MPU);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0;}
bool initIMU(){
 uint8_t who=0;
 if(!readBytes(0x75,&who,1) || (who&0x7E)!=0x68)return false;
 // Wake, PLL X gyro, sample divider=9 (100 Hz), ~20 Hz DLPF,
 // gyro +/-500 deg/s, accel +/-4g. Sensor output read back on every sample.
 return writeReg(0x6B,0x01)&&writeReg(0x19,9)&&writeReg(0x1A,4)&&writeReg(0x1B,8)&&writeReg(0x1C,8);
}
int16_t signedWord(const uint8_t *d){return (int16_t)(((uint16_t)d[0]<<8)|d[1]);}
void clearCalibrationSums(){calibrationCount=0;sumR=sumP=sumGX=sumGY=0;}
void startCalibration(){
 stopOutputs();fault="";calibrated=false;calibrating=false;
 imuOK=initIMU();
 if(!imuOK){fail("IMU initialization failed");return;}
 clearCalibrationSums();calibrating=true;lastSample=micros();lastGoodMs=millis();
 Serial.println("CALIBRATING: hold frame stationary and LEVEL for 4 seconds.");
}
void updateSensor(){
 uint32_t now=micros(),elapsed=now-lastSample;
 if(elapsed<SAMPLE_US)return;
 lastSample=now;
 float dt=elapsed*0.000001f;
 if(armed && dt>0.060f){fail("Control loop delayed: reset required");return;}
 dt=balance::limit(dt,0.001f,0.060f);
 uint8_t d[14];
 if(!readBytes(0x3B,d,14)){imuOK=false;if(!fault.length())fail("IMU read failed");return;}
 imuOK=true;lastGoodMs=millis();
 ax=signedWord(d)/8192.0f;ay=signedWord(d+2)/8192.0f;az=signedWord(d+4)/8192.0f;
 tempC=signedWord(d+6)/340.0f+36.53f;
 float gx=signedWord(d+8)/65.5f,gy=signedWord(d+10)/65.5f;
 float gz=signedWord(d+12)/65.5f;
 float magnitude=sqrtf(ax*ax+ay*ay+az*az);
 float accR=atan2f(ay,az)*180.0f/PI;
 float accP=atan2f(-ax,sqrtf(ay*ay+az*az))*180.0f/PI;
 if(!isfinite(accR)||!isfinite(accP)||magnitude<0.05f){if(!fault.length())fail("Invalid IMU sample");return;}
 if(calibrating){
  bool moving=fabsf(gx)>5 || fabsf(gy)>5 || fabsf(gz)>5 || magnitude<0.85f || magnitude>1.15f;
  if(calibrationCount>0 && (fabsf(balance::wrap(accR-previousCalR))>1.0f || fabsf(accP-previousCalP)>1.0f))moving=true;
  previousCalR=accR;previousCalP=accP;
  if(moving){clearCalibrationSums();return;}
  sumR+=accR;sumP+=accP;sumGX+=gx;sumGY+=gy;calibrationCount++;
  if(calibrationCount>=400){
   rollZero=sumR/calibrationCount;pitchZero=sumP/calibrationCount;
   gxBias=sumGX/calibrationCount;gyBias=sumGY/calibrationCount;
   filteredRoll=rollZero;filteredPitch=pitchZero;roll=pitch=rollRate=pitchRate=0;
   calibrating=false;calibrated=true;Serial.printf("Calibration OK. R0 %.2f P0 %.2f\n",rollZero,pitchZero);
  }
  return;
 }
 if(!calibrated || fault.length())return;
 float rawR=gx-gxBias,rawP=gy-gyBias;
 float rateAlpha=dt/(0.025f+dt);
 rollRate+=rateAlpha*(rawR-rollRate);pitchRate+=rateAlpha*(rawP-pitchRate);
 // Complementary filter. Gyro rates and accelerometer angles share the verified sensor axes.
 filteredRoll=balance::wrap(filteredRoll+rawR*dt);filteredPitch+=rawP*dt;
 if(magnitude>0.8f && magnitude<1.2f){
  float accWeight=dt/(0.5f+dt);
  filteredRoll=balance::wrap(filteredRoll+accWeight*balance::wrap(accR-filteredRoll));
  filteredPitch+=accWeight*(accP-filteredPitch);
 }
 roll=balance::wrap(filteredRoll-rollZero);pitch=filteredPitch-pitchZero;
 if(!isfinite(roll)||!isfinite(pitch)||!isfinite(rollRate)||!isfinite(pitchRate)){fail("Nonfinite attitude");return;}
 if(armed && (fabsf(roll)>TILT_LIMIT || fabsf(pitch)>TILT_LIMIT)){fail("Tilt exceeds 35 degrees");return;}
 if(armed && throttle>1000){
  // Freeze integrators at actuator limits to avoid accumulating unreachable corrections.
  bool saturated=false;for(int i=0;i<4;i++)if(motors[i]<=1000 || motors[i]>=balance::MOTOR_MAX_US)saturated=true;
  correctionR=rollPID.update(roll,rollRate,saturated?0:dt);
  correctionP=pitchPID.update(pitch,pitchRate,saturated?0:dt);
 }else{rollPID.reset();pitchPID.reset();correctionR=correctionP=0;}
 balance::mix(armed,throttle,correctionR,correctionP,trims,motors);writeMotors();
}
String stateName(){if(fault.length())return "FAULT";if(calibrating)return "CALIBRATING";return armed?"ARMED":"DISARMED";}
void sendState(){
 String s; s.reserve(850);
 s="{\"state\":\""+stateName()+"\",\"fault\":\""+fault+"\",\"armed\":"+(armed?String("true"):String("false"));
 s+=",\"calibrated\":"+String(calibrated?"true":"false")+",\"imuOK\":"+String(imuOK?"true":"false");
 s+=",\"progress\":"+String(calibrationCount/4)+",\"g\":"+String(generation);
 s+=",\"roll\":"+String(roll,2)+",\"pitch\":"+String(pitch,2)+",\"rollRate\":"+String(rollRate,2)+",\"pitchRate\":"+String(pitchRate,2);
 s+=",\"r0\":"+String(rollZero,2)+",\"p0\":"+String(pitchZero,2);
 s+=",\"throttle\":"+String(throttle)+",\"pidR\":"+String(correctionR,2)+",\"pidP\":"+String(correctionP,2);
 s+=",\"kp\":"+String(rollPID.kp,3)+",\"ki\":"+String(rollPID.ki,3)+",\"kd\":"+String(rollPID.kd,3);
 s+=",\"trims\":[";for(int i=0;i<4;i++){if(i)s+=",";s+=String(trims[i]);}
 s+="],\"motors\":[";for(int i=0;i<4;i++){if(i)s+=",";s+=String(motors[i]);}
 s+="],\"accelG\":["+String(ax,3)+","+String(ay,3)+","+String(az,3)+"],\"temp\":"+String(tempC,1);
 s+=",\"sampleAge\":"+String(millis()-lastGoodMs)+"}";
 server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",s);
}
void errorReply(int code,const char *msg){server.send(code,"text/plain",msg);}
bool parseNumber(const char *name,float lo,float hi,float &value,bool integerOnly=false){
 if(!server.hasArg(name))return false;
 String s=server.arg(name);if(!s.length() || s.length()>16)return false;
 char *end=nullptr;value=strtof(s.c_str(),&end);
 return end!=s.c_str() && *end=='\0' && isfinite(value) && value>=lo && value<=hi && (!integerOnly || floorf(value)==value);
}
bool sessionOK(){
 if(!armed || server.arg("g")!=String(generation)){errorReply(409,"Session stopped. ARM again.");return false;}
 if(millis()-lastBeat>HEARTBEAT_MS){stopOutputs();errorReply(409,"Heartbeat expired. ARM again.");return false;}
 return true;
}
void routes(){
 server.on("/",HTTP_GET,[]{server.sendHeader("Cache-Control","no-store");server.send_P(200,"text/html",DASHBOARD_HTML);});
 server.on("/api/state",HTTP_GET,sendState);
 server.on("/api/stop",HTTP_POST,[]{stopOutputs();sendState();});
 server.on("/api/reset",HTTP_POST,[]{startCalibration();sendState();});
 server.on("/api/arm",HTTP_POST,[]{
  if(server.arg("g")!=String(generation)){errorReply(409,"Stale ARM request. Refresh status and try again.");return;}
  if(armed){errorReply(409,"Already armed");return;}
  if(!pwmOK || fault.length() || !imuOK || !calibrated || calibrating || millis()-bootMs<5000 || millis()-lastGoodMs>100){errorReply(409,"Wait for calibration / reset FAULT first");return;}
  if(fabsf(roll)>8 || fabsf(pitch)>8 || fabsf(rollRate)>10 || fabsf(pitchRate)>10){errorReply(409,"Hold level and still before ARM");return;}
  stopOutputs();armed=true;lastBeat=millis();sendState();
 });
 server.on("/api/beat",HTTP_POST,[]{if(!sessionOK())return;lastBeat=millis();sendState();});
 server.on("/api/throttle",HTTP_POST,[]{
  if(!sessionOK())return;float value;
  if(!parseNumber("v",1000,balance::BASE_MAX_US,value,true)){errorReply(400,"Throttle must be an integer 1000..1300");return;}
  throttle=(int)value;lastBeat=millis();
  if(throttle==1000){for(int i=0;i<4;i++)trims[i]=0;rollPID.reset();pitchPID.reset();correctionR=correctionP=0;balance::mix(false,1000,0,0,trims,motors);writeMotors();}
  sendState();
 });
 server.on("/api/trim",HTTP_POST,[]{
  if(!sessionOK())return;float id,value;
  if(!parseNumber("id",1,4,id,true)||!parseNumber("v",-40,40,value,true)){errorReply(400,"Trim id 1..4; value -40..40 us");return;}
  trims[(int)id-1]=(int)value;lastBeat=millis();sendState();
 });
 server.on("/api/gains",HTTP_POST,[]{
  if(armed || calibrating){errorReply(409,"STOP before editing gains");return;}
  float kp,ki,kd;
  if(!parseNumber("kp",0,5,kp)||!parseNumber("ki",0,1,ki)||!parseNumber("kd",0,1,kd)){errorReply(400,"Gain limits Kp 0..5, Ki/Kd 0..1");return;}
  rollPID.kp=pitchPID.kp=kp;rollPID.ki=pitchPID.ki=ki;rollPID.kd=pitchPID.kd=kd;
  rollPID.reset();pitchPID.reset();sendState();
 });
}
void drawHorizon(){
 // Clip the line in a 60x34 region; fixed center aircraft marker.
 float angle=-roll*PI/180.0f;
 float cy=29+balance::limit(pitch,-25,25)*0.5f;
 for(int x=1;x<59;x++){int y=(int)lroundf(cy+tanf(angle)*(x-30));if(y>=14 && y<=46)oled.drawPixel(x,y,SSD1306_WHITE);}
 oled.drawRect(0,13,60,35,SSD1306_WHITE);oled.drawFastHLine(22,30,17,SSD1306_WHITE);oled.drawFastVLine(30,27,7,SSD1306_WHITE);
}
void updateOLED(){
 if(!oledOK || millis()-lastOLED<200)return;lastOLED=millis();
 oled.clearDisplay();oled.setTextColor(SSD1306_WHITE);oled.setTextSize(1);oled.setCursor(0,0);oled.print("ESP32 PID");oled.setCursor(86,0);oled.print(fault.length()?"FAULT":calibrating?"CAL":armed?"ARM":"OFF");
 if(calibrating){oled.setCursor(0,20);oled.print("Hold LEVEL + STILL");oled.setCursor(0,35);oled.print("Calibrating ");oled.print(calibrationCount/4);oled.print("%");}
 else if(fault.length()){oled.setCursor(0,17);oled.print(fault);oled.setCursor(0,48);oled.print("STOP / RESET");}
 else if((millis()/2500)%2==0){
  drawHorizon();oled.setCursor(65,16);oled.print("R ");oled.print(roll,1);oled.setCursor(65,29);oled.print("P ");oled.print(pitch,1);oled.setCursor(65,42);oled.print("T ");oled.print(throttle);
 }else{
  for(int i=0;i<4;i++){oled.setCursor((i%2)*65,17+(i/2)*18);oled.print("M");oled.print(i+1);oled.print(":");oled.print(motors[i]);}
  oled.setCursor(0,47);oled.print("FR RR FL RL / us");
 }
 oled.setCursor(0,57);oled.print("192.168.4.1");oled.setCursor(92,57);oled.print(WiFi.softAPgetStationNum()?"LINK":"WAIT");oled.display();
}
void setup(){
 Serial.begin(115200);
 for(int i=0;i<4;i++){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if(!ledcAttach(MOTOR_PINS[i],50,16))pwmOK=false;
#else
  if(ledcSetup(i,50,16)==0)pwmOK=false;ledcAttachPin(MOTOR_PINS[i],i);
#endif
 }
 stopOutputs();bootMs=millis();
 Wire.begin(21,22);Wire.setClock(400000);Wire.setTimeOut(10);
 Wire.beginTransmission(0x3C);
 if(Wire.endTransmission()==0)oledOK=oled.begin(SSD1306_SWITCHCAPVCC,0x3C,true,false);
 Wire.setClock(400000);
 WiFi.mode(WIFI_AP);WiFi.softAP("ESP32-Drone-Test","drone12345",1,0,1);WiFi.setSleep(false);
 routes();server.begin();
 if(pwmOK)startCalibration();else fail("PWM initialization failed");
 Serial.println("WiFi ESP32-Drone-Test / drone12345 / http://192.168.4.1");
 Serial.println("Serial: ! or 0 STOP; c recalibrate while level.");
}
void loop(){
 if(armed && (millis()-lastBeat>HEARTBEAT_MS || WiFi.softAPgetStationNum()==0))stopOutputs();
 if(armed && millis()-lastGoodMs>100)fail("IMU sample timeout");
 updateSensor();
 if(Serial.available()){char c=Serial.read();if(c=='!'||c=='0')stopOutputs();else if(c=='c'||c=='C')startCalibration();}
 server.handleClient();updateSensor();updateOLED();
 if(millis()-lastLog>500){lastLog=millis();Serial.printf("%s R:%+.1f P:%+.1f T:%d | FR:%d RR:%d FL:%d RL:%d\n",stateName().c_str(),roll,pitch,throttle,motors[0],motors[1],motors[2],motors[3]);}
 delay(1);
}

