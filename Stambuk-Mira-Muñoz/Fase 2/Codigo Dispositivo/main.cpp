// ================= Librerías =================
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024
#define TINY_GSM_DEBUG SerialMon

#define SerialMon Serial
#define SerialAT ModemSerial

#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <cstring>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

#include "secrets.h"

// ================= Pines =================
#define GPS_RX 21
#define GPS_TX 20
#define MPU_SDA 8
#define MPU_SCL 9
#define MODEM_RX_PIN 5   
#define MODEM_TX_PIN 6   
#define BUZZER_PIN 7
#define NEOPIXEL_PIN 4
#define BATTERY_ADC_PIN 0  

// ================= Configuración general =================
#define BUZZER_FREQ 2200
#define BUZZER_BEEP_MS 350
#define G_SAMPLE_INTERVAL_MS 20
#define FALL_FREEFALL_G_MAX 0.65f      
#define FALL_FREEFALL_MIN_MS 80       
#define FALL_IMPACT_G_MIN 1.8f         
#define FALL_IMPACT_WINDOW_MS 500      
#define FALL_EVENT_COOLDOWN 4000       
#define FALL_GYRO_ROTATION_THRESHOLD 120.0f  
#define FALL_GYRO_TOTAL_ROTATION_MIN 45.0f   

static const uint8_t MIN_SATS_GOOD=5;
static const uint16_t MAX_HDOP_GOOD=300; 

// ================= Configuración de batería =================
#define BATTERY_MIN_VOLTAGE 3.0f    
#define BATTERY_MAX_VOLTAGE 4.2f    
#define BATTERY_DIVIDER_RATIO 0.5f  
#define BATTERY_CRITICAL_PERCENT 20  
#define BATTERY_ADC_SAMPLES 10      
#define BATTERY_REPORT_INTERVAL_MS 1800000  // 30 minutos
#define BATTERY_ADC_RESOLUTION 4095 
#define BATTERY_REFERENCE_VOLTAGE 3.3f

// ================= Módem / Red =================
#define SIM_APN            "internet"    
#define SIM_PIN            ""             
#define SIM_DEBUG          1              
#define ENABLE_AT_BRIDGE   1              

// TinyGSM configuración
#define GSM_DEBUG          SerialMon     
#define SerialMon          Serial                    

// ================= Firebase (Realtime Database REST) =================
#ifdef __has_include
  #if __has_include(<secrets.h>)
    #include <secrets.h>
  #endif
#endif
#ifndef FIREBASE_HOST
  #define FIREBASE_HOST   ""
#endif
#ifndef FIREBASE_AUTH
  #define FIREBASE_AUTH   ""  
#endif
#ifndef FIREBASE_DEVICE
  #define FIREBASE_DEVICE ""
#endif

static String fbUrl(const String &path){
  String url = String("https://") + FIREBASE_HOST + path + ".json";
  if(strlen(FIREBASE_AUTH)){
    url += String("?auth=") + FIREBASE_AUTH;
  }
  return url;
}

static String fbPathAlerts(){ return String("/devices/") + FIREBASE_DEVICE + "/alerts"; }
static String fbPathTracks(){ return String("/devices/") + FIREBASE_DEVICE + "/tracks"; }
static String fbPathCmd(){ return String("/devices/") + FIREBASE_DEVICE + "/cmd"; }
static String fbPathBattery(){ return String("/devices/") + FIREBASE_DEVICE + "/battery"; }

// ================= Hardware y constantes MPU =================
static const uint8_t MPU_ADDR = 0x68;
static const float   ACC_LSB_PER_G = 16384.0f;
static const float   GYRO_LSB_PER_DEG = 131.0f;

// ================= Prototipos =================
bool initModem();
void checkNetwork();
bool simHttpGet(const char* url, String &outBody);
bool simHttpPost(const String& url, const char* data);
void pollRemoteCommand();
void sendTrackingUpdate();
void sendAlert(float gmag);
bool initMPU();
bool readAccelG(float &gx,float &gy,float &gz);
float readBatteryVoltage();
int getBatteryPercentage(float voltage);
void sendBatteryUpdate();
void checkBatteryLevel();

// ================= Instancias =================
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); 
HardwareSerial ModemSerial(1);            
TinyGPSPlus gps;
Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

TinyGsm modem(ModemSerial);
TinyGsmClient client(modem);

// ================= Estado global =================
unsigned long lastPrint=0, lastByteMs=0;
uint8_t lastLedMode=0xFF;
bool mpuOk=false, mpuCalibrated=false;
int32_t ax_off=0, ay_off=0, az_off=0;
int32_t gx_off=0, gy_off=0, gz_off=0;
uint16_t calibSamplesTarget=200, calibCollected=0;
unsigned long lastGSample=0, lastFallEvent=0;
float gAvg=1.0f; bool gAvgInit=false;
float gyroMagnitude=0.0f; 
float totalRotation=0.0f;  
float maxGyroInFall=0.0f;  
float maxGInFall=0.0f;     
float minGInFall=10.0f;    
bool impactDetectedInSequence=false; 
enum FallState { FS_IDLE, FS_FREEFALL, FS_WAIT_IMPACT };
FallState fallState=FS_IDLE;
unsigned long freefallStart=0, freefallEnd=0;
unsigned long buzzerUntil=0; uint8_t buzzerChannel=0;

// Red
bool trackingActive=false;
unsigned long lastTrackSent=0;
const uint32_t TRACK_INTERVAL_MS=15000;
unsigned long lastCommandPoll=0;
const uint32_t COMMAND_POLL_INTERVAL_MS=5000;
bool networkConnected=false;             
unsigned long lastNetworkCheck=0;   
const uint32_t NETWORK_CHECK_INTERVAL_MS=20000;

// Batería
unsigned long lastBatteryReport=0;
bool batteryLowAlertSent=false;

// ================= Utilidades =================
inline void setColor(uint8_t r,uint8_t g,uint8_t b){ led.setPixelColor(0, led.Color(r,g,b)); led.show(); }
inline bool goodFix(){ return gps.location.isValid() && gps.satellites.isValid() && gps.satellites.value()>=MIN_SATS_GOOD && gps.hdop.isValid() && gps.hdop.value()<=MAX_HDOP_GOOD; }

// ================= Buzzer =================
void buzzerStart(uint16_t ms=BUZZER_BEEP_MS){ ledcWrite(buzzerChannel,512); buzzerUntil=millis()+ms; }
void buzzerUpdate(){ if(buzzerUntil && millis()>buzzerUntil){ ledcWrite(buzzerChannel,0); buzzerUntil=0; } }

// ================= MPU =================
bool mpuWrite(uint8_t reg,uint8_t val){ Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); return Wire.endTransmission()==0; }
bool mpuReadMulti(uint8_t startReg,uint8_t count,uint8_t *buf){ Wire.beginTransmission(MPU_ADDR); Wire.write(startReg); if(Wire.endTransmission(false)!=0) return false; uint8_t n=Wire.requestFrom(MPU_ADDR,count); if(n!=count) return false; for(uint8_t i=0;i<count;i++) buf[i]=Wire.read(); return true; }
bool initMPU(){ 
  Wire.begin(MPU_SDA, MPU_SCL, 400000); 
  delay(50); 
  if(!mpuWrite(0x6B,0x00)) return false; 
  delay(10); 
  if(!mpuWrite(0x1C,0x00)) return false; 
  if(!mpuWrite(0x1B,0x00)) return false; 
  mpuWrite(0x1A,0x03); 
  return true; 
}

bool readAccelRaw(int16_t &rx,int16_t &ry,int16_t &rz){ 
  uint8_t d[6]; 
  if(!mpuReadMulti(0x3B,6,d)) return false; 
  rx=(int16_t)((d[0]<<8)|d[1]); 
  ry=(int16_t)((d[2]<<8)|d[3]); 
  rz=(int16_t)((d[4]<<8)|d[5]); 
  return true; 
}

bool readGyroRaw(int16_t &rx,int16_t &ry,int16_t &rz){ 
  uint8_t d[6]; 
  if(!mpuReadMulti(0x43,6,d)) return false; 
  rx=(int16_t)((d[0]<<8)|d[1]); 
  ry=(int16_t)((d[2]<<8)|d[3]); 
  rz=(int16_t)((d[4]<<8)|d[5]); 
  return true; 
}

bool readAccelG(float &gx,float &gy,float &gz){ 
  int16_t rx,ry,rz; 
  if(!readAccelRaw(rx,ry,rz)) return false; 
  gx=(rx-ax_off)/ACC_LSB_PER_G; 
  gy=(ry-ay_off)/ACC_LSB_PER_G; 
  gz=(rz-az_off)/ACC_LSB_PER_G; 
  return true; 
}

bool readGyroDPS(float &gx,float &gy,float &gz){ 
  int16_t rx,ry,rz; 
  if(!readGyroRaw(rx,ry,rz)) return false; 
  gx=(rx-gx_off)/GYRO_LSB_PER_DEG; 
  gy=(ry-gy_off)/GYRO_LSB_PER_DEG; 
  gz=(rz-gz_off)/GYRO_LSB_PER_DEG; 
  return true; 
}

void handleMPU(){
  if(!mpuOk) return;
  if(!mpuCalibrated){
    int16_t ax,ay,az,gx,gy,gz; 
    if(readAccelRaw(ax,ay,az) && readGyroRaw(gx,gy,gz)){
      ax_off+=ax; 
      ay_off+=ay; 
      gx_off+=gx; 
      gy_off+=gy; 
      gz_off+=gz;
      
      calibCollected++; 
      if(calibCollected>=calibSamplesTarget){ 
        ax_off/=calibCollected; 
        ay_off/=calibCollected; 
        
        gx_off/=calibCollected; 
        gy_off/=calibCollected; 
        gz_off/=calibCollected;
        
        float test_gx, test_gy, test_gz;
        if(readAccelG(test_gx, test_gy, test_gz)){
          float test_gmag = sqrtf(test_gx*test_gx + test_gy*test_gy + test_gz*test_gz);
          
          if(test_gmag < 0.8f || test_gmag > 1.2f){
            Serial.println("\n ADVERTENCIA: Calibración Malfuncionando");
            Serial.print("   |G| después de calibrar: "); Serial.print(test_gmag,3); 
            Serial.println("g (esperado: ~1.0g)");
            Serial.println("   Asegúrate de que el dispositivo esté en reposo\n");
          }
        }
        
        mpuCalibrated=true; 
        Serial.println("\n========== MPU CALIBRADO (accel + gyro) ==========");
        Serial.print("Offsets accel: X="); Serial.print(ax_off);
        Serial.print(" Y="); Serial.print(ay_off);
        Serial.print(" Z="); Serial.print(az_off); Serial.println(" (Z mantiene gravedad)");
        Serial.print("Offsets gyro:  X="); Serial.print(gx_off);
        Serial.print(" Y="); Serial.print(gy_off);
        Serial.print(" Z="); Serial.println(gz_off);
        Serial.println("==================================================\n");
      }
    }
    return;
  }
  unsigned long now=millis(); 
  if(now-lastGSample < G_SAMPLE_INTERVAL_MS) return; 
  
  float dt = (now - lastGSample) / 1000.0f; 
  lastGSample=now;
  
  float ax,ay,az; 
  float gx,gy,gz;
  if(!readAccelG(ax,ay,az)) return;
  if(!readGyroDPS(gx,gy,gz)) return;
  
  float gmag = sqrtf(ax*ax+ay*ay+az*az);
  gyroMagnitude = sqrtf(gx*gx+gy*gy+gz*gz); 
  
  if(!gAvgInit){ gAvg=gmag; gAvgInit=true; } else gAvg=gAvg*0.9f+gmag*0.1f;
  
  unsigned long ms=now;
  switch(fallState){
    case FS_IDLE:
      totalRotation = 0.0f; 
      maxGyroInFall = 0.0f;
      maxGInFall = 0.0f;
      minGInFall = 10.0f;
      impactDetectedInSequence = false;
      
      if(gmag < FALL_FREEFALL_G_MAX && gmag >= 0.15f){ 
        freefallStart=ms; 
        fallState=FS_FREEFALL;
        minGInFall = gmag;
        maxGInFall = gmag;
      }
      break;
      
    case FS_FREEFALL:
      totalRotation += gyroMagnitude * dt;
      if(gyroMagnitude > maxGyroInFall) maxGyroInFall = gyroMagnitude;
      if(gmag < minGInFall) minGInFall = gmag;
      if(gmag > maxGInFall) maxGInFall = gmag;

      if(gmag >= FALL_IMPACT_G_MIN && ms - freefallStart >= FALL_FREEFALL_MIN_MS){
        freefallEnd=ms;
        impactDetectedInSequence = true;
        fallState=FS_WAIT_IMPACT;
      }
      else if(gmag < FALL_FREEFALL_G_MAX){ 
        if(ms - freefallStart >= FALL_FREEFALL_MIN_MS){ 
          freefallEnd=ms; 
          fallState=FS_WAIT_IMPACT;
        }
      } else {
        if(totalRotation < FALL_GYRO_TOTAL_ROTATION_MIN){
          fallState=FS_IDLE;
        } else {
          freefallEnd=ms;
          fallState=FS_WAIT_IMPACT;
        }
      }
      break;
      
    case FS_WAIT_IMPACT:
      totalRotation += gyroMagnitude * dt;
      if(gyroMagnitude > maxGyroInFall) maxGyroInFall = gyroMagnitude;
      if(gmag > maxGInFall) maxGInFall = gmag;
      
      if(impactDetectedInSequence || gmag >= FALL_IMPACT_G_MIN){
        if(!impactDetectedInSequence) {
          impactDetectedInSequence = true;
        }
      }
      
      if(impactDetectedInSequence && (ms - freefallEnd >= 100 || (maxGInFall >= FALL_IMPACT_G_MIN && gmag < maxGInFall * 0.7))){
        bool validFall = false;
        String validationReason = "";
        
        if(totalRotation >= FALL_GYRO_TOTAL_ROTATION_MIN){
          validFall = true;
          validationReason = "Rotación total suficiente";
        }
        else if(gyroMagnitude >= FALL_GYRO_ROTATION_THRESHOLD){
          validFall = true;
          validationReason = "Rotación instantánea alta";
        }
        
        if(validFall && ms - lastFallEvent > FALL_EVENT_COOLDOWN){
          lastFallEvent=ms;     
          
          Serial.println("\n Caida Detectada");

          buzzerStart(100);
          buzzerStart(150); 
          buzzerStart(100);  
          sendAlert(maxGInFall);
        }
        
        fallState=FS_IDLE;
      } else if (ms - freefallEnd > FALL_IMPACT_WINDOW_MS) {
        fallState=FS_IDLE;
      }
      break;
  }
}

// ================= LED =================
void updateLED(){
  uint32_t now=millis();
  uint8_t m = (now - lastByteMs > 3000)?0:(goodFix()?3:(gps.location.isValid()||(gps.satellites.isValid()&&gps.satellites.value()>0))?2:1);
  if(m==lastLedMode) return; lastLedMode=m;
  switch(m){
    case 0: setColor(40,0,0); break;   // rojo
    case 1: setColor(0,0,40); break;   // azul
    case 2: setColor(25,0,40); break;  // morado
    case 3: setColor(0,40,0); break;   // verde
  }
}

// ================= Módem (TinyGSM A7670SA) =================
bool initModem(){
  Serial.println("[MODEM] Iniciando A7670SA con TinyGSM...");
  
  if(!modem.init()){
    Serial.println("[MODEM] Init fallo");
    return false;
  }
  
  String modemInfo = modem.getModemInfo();
  Serial.print("[MODEM] Info: "); Serial.println(modemInfo);
  
  if(strlen(SIM_PIN) > 0){
    if(!modem.simUnlock(SIM_PIN)){
      Serial.println("[MODEM] SIM unlock fallo");
      return false;
    }
  }
  
  Serial.println("[MODEM] Esperando registro en red...");
  if(!modem.waitForNetwork(60000)){
    Serial.println("[MODEM] Registro en red fallo");
    return false;
  }
  
  Serial.println("[MODEM] Conectando GPRS...");
  if(!modem.gprsConnect(SIM_APN)){
    Serial.println("[MODEM] GPRS connect fallo");
    return false;
  }
  
  Serial.println("[MODEM] Configurando SSL...");
  
  modem.sendAT("+CSSLCFG=\"sslversion\",0,4");  
  if(!modem.waitResponse(5000, "OK")) {
    Serial.println("[MODEM] CSSLCFG sslversion fallo");
  }
  
  modem.sendAT("+CSSLCFG=\"authmode\",0,0");   
  if(!modem.waitResponse(5000, "OK")) {
    Serial.println("[MODEM] CSSLCFG authmode fallo");
  }
  
  modem.sendAT("+CSSLCFG=\"protocol\",0,0");    
  if(!modem.waitResponse(5000, "OK")) {
    Serial.println("[MODEM] CSSLCFG protocol fallo");
  }
  
  modem.sendAT("+CSSLCFG=\"enableSNI\",0,1");
  if(!modem.waitResponse(5000, "OK")) {
    Serial.println("[MODEM] CSSLCFG enableSNI fallo");
  }
  
  Serial.println("[MODEM] Conectado exitosamente");
  networkConnected = true;
  return true;
}

void checkNetwork(){
  if(millis() - lastNetworkCheck < NETWORK_CHECK_INTERVAL_MS) return;
  lastNetworkCheck = millis();
  
  if(!modem.isNetworkConnected()){
    Serial.println("[MODEM] Red perdida, reconectando...");
    networkConnected = false;
    
    if(modem.waitForNetwork(30000) && modem.gprsConnect(SIM_APN)){
      Serial.println("[MODEM] Reconectado");
      networkConnected = true;
    }
  }
}

// ================= HTTP Client (TinyGSM + ArduinoHttpClient) =================
bool simHttpGet(const char* url, String &outBody){
  if(!networkConnected) {
    if(!initModem()) {
      Serial.println("[HTTP] Módem no conectado");
      return false;
    }
  }
  
  for(int attempt = 0; attempt < 3; attempt++) {
    Serial.print("[HTTP] Intento "); Serial.print(attempt+1); Serial.println("...");
    
    modem.sendAT("+HTTPTERM");
    modem.waitResponse(2000);
    delay(500);
    
    modem.sendAT("+HTTPINIT");
    if(!modem.waitResponse(5000, "OK")) {
      Serial.println("[HTTP] HTTPINIT fallo");
      continue;
    }
    
    modem.sendAT("+HTTPSSL=1");
    if(!modem.waitResponse(3000, "OK")) {
      Serial.println("[HTTP] HTTPSSL fallo");
      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      continue;
    }
    
    modem.sendAT("+HTTPPARA=\"USERDATA\",\"Elder-Guardian/1.0\"");
    modem.waitResponse(2000);
    
    modem.sendAT("+HTTPPARA=\"REDIR\",\"1\"");  
    modem.waitResponse(2000);
    
    modem.sendAT("+HTTPPARA=\"URL\",\"", url, "\"");
    String paraResponse = "";
    int8_t paraRet = modem.waitResponse(3000, paraResponse);
    
    if(paraRet <= 0 || paraResponse.indexOf("OK") < 0) {
      Serial.println("[HTTP] HTTPPARA URL fallo");
      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      continue;
    }
    
    modem.sendAT("+HTTPACTION=0");
    
    String actionResponse = "";
    unsigned long startTime = millis();
    bool actionComplete = false;
    
    while(millis() - startTime < 30000 && !actionComplete) {
      if(modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        if(line.length() > 0) {
          actionResponse += line + "\n";
          if(line.indexOf("+HTTPACTION:") >= 0) {
            actionComplete = true;
            break;
          }
        }
      }
      delay(100);
    }
    
    bool actionSuccess = false;
    int dataSize = 0;
    
    if(actionComplete && actionResponse.indexOf("+HTTPACTION: 0,200") >= 0) {
      actionSuccess = true;
      
      int sizeStart = actionResponse.lastIndexOf(',') + 1;
      if(sizeStart > 0) {
        String sizeStr = actionResponse.substring(sizeStart);
        sizeStr.trim();
        dataSize = sizeStr.toInt();
        Serial.print("[HTTP] Datos disponibles: "); Serial.print(dataSize); Serial.println(" bytes");
      }
    } else if(actionComplete) {
      Serial.print("[HTTP] HTTPACTION fallo - respuesta: ");
      Serial.println(actionResponse);
    } else {
      Serial.println("[HTTP] HTTPACTION timeout");
    }
    
    if(!actionSuccess) {
      Serial.println("[HTTP] HTTPACTION fallo o status no exitoso");
      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      continue;
    }
    
    delay(1000); 
    
    modem.sendAT("+HTTPREAD=0,", dataSize);
    
    String response;
    unsigned long startTimeRead = millis();
    bool foundHttpRead = false;
    while (millis() - startTimeRead < 10000) {
      if (modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        if (line.startsWith("+HTTPREAD:")) {
          foundHttpRead = true;
          break; 
        }
      }
    }

    if (foundHttpRead) {
      outBody = "";
      char buffer[256];
      int bytesRead = 0;
      startTimeRead = millis();
      while(bytesRead < dataSize && millis() - startTimeRead < 10000){
        int avail = modem.stream.available();
        if(avail > 0){
          int toRead = min(avail, (int)sizeof(buffer)-1);
          toRead = min(toRead, dataSize - bytesRead);
          int read = modem.stream.readBytes(buffer, toRead);
          buffer[read] = '\0';
          outBody += buffer;
          bytesRead += read;
        }
        delay(10);
      }
      
      Serial.print("[HTTP] DEBUG - Cuerpo leído: '"); Serial.print(outBody); Serial.println("'");

      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      Serial.println("[HTTP] GET exitoso");
      return true;
    } else {
      Serial.println("[HTTP] HTTPREAD fallo - no se encontró prompt");
    }
    
    Serial.print("[HTTP] GET fallo intento "); Serial.println(attempt+1);
    modem.sendAT("+HTTPTERM");
    modem.waitResponse(2000);
    delay(2000); 
  }
  
  return false;
}

bool simHttpPost(const String& url, const char* data){
  if(!networkConnected) {
    if(!initModem()) {
      Serial.println("[HTTP] Módem no conectado");
      return false;
    }
  }
  
  Serial.print("[HTTP] DEBUG - POST URL: ");
  Serial.println(url);
  Serial.print("[HTTP] DEBUG - POST Data: ");
  Serial.println(data);
  
  int dataLen = strlen(data);
  Serial.print("[HTTP] DEBUG - Data Length: ");
  Serial.println(dataLen);
  
  for(int attempt = 0; attempt < 3; attempt++) {
    Serial.print("[HTTP] DEBUG - POST Intento "); Serial.print(attempt+1); Serial.println(" con comandos AT...");
    
    if(attempt > 0) {
      Serial.println("[HTTP] Verificando estado del módem...");
      modem.sendAT();
      if(!modem.waitResponse(3000, "OK")) {
        Serial.println("[HTTP] Módem no responde, saltando intento");
        continue;
      }
    }
    
    modem.sendAT("+HTTPTERM");
    modem.waitResponse(2000);
    delay(500);
    
    modem.sendAT("+HTTPINIT");
    if(!modem.waitResponse(5000, "OK")) {
      Serial.println("[HTTP] HTTPINIT fallo");
      continue;
    }
    
    modem.sendAT("+HTTPSSL=1");
    if(!modem.waitResponse(3000, "OK")) {
      Serial.println("[HTTP] HTTPSSL fallo");
      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      continue;
    }
    
    modem.sendAT("+HTTPPARA=\"URL\",\"", url, "\"");
    if(!modem.waitResponse(3000, "OK")) {
      Serial.println("[HTTP] HTTPPARA URL fallo");
      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      continue;
    }
    
    modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
    if(!modem.waitResponse(3000, "OK")) {
      Serial.println("[HTTP] HTTPPARA CONTENT fallo");
      modem.sendAT("+HTTPTERM");
      modem.waitResponse(2000);
      continue;
    }
    
    modem.sendAT("+HTTPPARA=\"USERDATA\",\"Elder-Guardian/1.0\"");
    modem.waitResponse(2000);
    
    modem.sendAT("+HTTPPARA=\"REDIR\",\"1\"");  
    modem.waitResponse(2000);
    
    String dataCmd = "+HTTPDATA=" + String(dataLen) + ",10000";
    modem.sendAT(dataCmd.c_str());
    
    String downloadResponse = "";
    bool downloadPromptFound = false;
    unsigned long downloadStart = millis();
    
    while(millis() - downloadStart < 5000 && !downloadPromptFound) {
      if(modem.stream.available()) {
        String line = modem.stream.readStringUntil('\n');
        line.trim();
        downloadResponse += line + "\n";
        
        Serial.print("[HTTP] DEBUG - Línea recibida: '");
        Serial.print(line);
        Serial.println("'");
        
        if(line.indexOf("DOWNLOAD") >= 0) {
          downloadPromptFound = true;
          Serial.println("[HTTP] DOWNLOAD prompt encontrado!");
          break;
        }
      }
      delay(10);
    }
    
    if(downloadPromptFound) {
      Serial.println("[HTTP] DOWNLOAD prompt recibido, enviando datos...");
      
      modem.stream.print(data);
      modem.stream.flush();
      
      String httpDataResponse = "";
      bool httpDataOk = false;
      unsigned long httpDataStart = millis();
      
      while(millis() - httpDataStart < 15000 && !httpDataOk) {
        if(modem.stream.available()) {
          String line = modem.stream.readStringUntil('\n');
          line.trim();
          httpDataResponse += line + "\n";
          
          Serial.print("[HTTP] DEBUG - HTTPDATA Resp: '");
          Serial.print(line);
          Serial.println("'");
          
          if(line.indexOf("OK") >= 0) {
            httpDataOk = true;
            Serial.println("[HTTP] HTTPDATA completado exitosamente");
            break;
          }
        }
        delay(10);
      }
      
      if(httpDataOk) {
        
        modem.sendAT("+HTTPACTION=1");
        
        String actionResponse = "";
        unsigned long startTime = millis();
        bool actionComplete = false;
        
        while(millis() - startTime < 30000 && !actionComplete) {
          if(modem.stream.available()) {
            String line = modem.stream.readStringUntil('\n');
            line.trim();
            if(line.length() > 0) {
              actionResponse += line + "\n";
              Serial.print("[HTTP] Response line: ");
              Serial.println(line);
              
              if(line.indexOf("+HTTPACTION:") >= 0) {
                actionComplete = true;
                break;
              }
            }
          }
          delay(100);
        }
        
        if(actionComplete && (actionResponse.indexOf("+HTTPACTION: 1,200") >= 0 || 
                             actionResponse.indexOf("+HTTPACTION: 1,201") >= 0)) {
          modem.sendAT("+HTTPTERM");
          modem.waitResponse(2000);
          Serial.println("[HTTP] DEBUG - POST exitoso con comandos AT");
          return true;
        } else if(actionComplete) {
          Serial.print("[HTTP] POST fallo - respuesta completa: ");
          Serial.println(actionResponse);
        } else {
          Serial.println("[HTTP] POST timeout - sin respuesta HTTPACTION");
        }
        
      } else {
        Serial.print("[HTTP] HTTPDATA fallo - respuesta: ");
        Serial.println(httpDataResponse);
      }
    } else {
      Serial.print("[HTTP] HTTPDATA download prompt fallo - respuesta: ");
      Serial.println(downloadResponse);
    }
    
    Serial.print("[HTTP] POST fallo intento "); Serial.println(attempt+1);
    
    modem.sendAT("+HTTPTERM");
    modem.waitResponse(3000);
    
    while(modem.stream.available()) {
      modem.stream.read();
    }
    
    delay(3000);
  }
  
  Serial.println("[HTTP] Todos los intentos POST fallaron");
  return false;
}

// ================= Monitoreo de Batería =================
float readBatteryVoltage(){
  analogSetAttenuation(ADC_11db); 
  long adcSum = 0;
  for(int i = 0; i < BATTERY_ADC_SAMPLES; i++){
    adcSum += analogRead(BATTERY_ADC_PIN);
    delay(10);
  }
  
  float adcAverage = (float)adcSum / BATTERY_ADC_SAMPLES;
  float pinVoltage = (adcAverage / BATTERY_ADC_RESOLUTION) * BATTERY_REFERENCE_VOLTAGE;
  float batteryVoltage = pinVoltage / BATTERY_DIVIDER_RATIO;
  
  Serial.print("[BAT] ADC: "); Serial.print(adcAverage);
  Serial.print(" PinV: "); Serial.print(pinVoltage, 3);
  Serial.print("V BatV: "); Serial.print(batteryVoltage, 3); Serial.println("V");
  
  return batteryVoltage;
}

int getBatteryPercentage(float voltage){
  if(voltage >= BATTERY_MAX_VOLTAGE) return 100;
  if(voltage <= BATTERY_MIN_VOLTAGE) return 0;
  
  if(voltage > 4.0f) {
    // 4.2V-4.0V = 100%-90% 
    return (int)(90 + ((voltage - 4.0f) / 0.2f) * 10);
  } else if(voltage > 3.7f) {
    // 4.0V-3.7V = 90%-50% 
    return (int)(50 + ((voltage - 3.7f) / 0.3f) * 40);
  } else if(voltage > 3.4f) {
    // 3.7V-3.4V = 50%-20% 
    return (int)(20 + ((voltage - 3.4f) / 0.3f) * 30);
  } else {
    // 3.4V-3.0V = 20%-0% 
    return (int)(((voltage - BATTERY_MIN_VOLTAGE) / 0.4f) * 20);
  }
}

void sendBatteryUpdate(){
  if(!networkConnected) return;
  
  float voltage = readBatteryVoltage();
  int percentage = getBatteryPercentage(voltage);
  
  char body[256];
  if(gps.date.isValid() && gps.time.isValid()){
    snprintf(body, sizeof(body), 
             "{\"voltage\":%.3f,\"percentage\":%d,\"date\":\"%02d/%02d/%04d\",\"time\":\"%02d:%02d:%02d\"}", 
             voltage, percentage,
             gps.date.day(), gps.date.month(), gps.date.year(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    snprintf(body, sizeof(body), 
             "{\"voltage\":%.3f,\"percentage\":%d,\"timestamp\":%lu}", 
             voltage, percentage, millis());
  }
  
  String url = fbUrl(fbPathBattery());
  if(simHttpPost(url.c_str(), body)) {
    Serial.print("[BAT] Enviado: "); Serial.print(percentage); Serial.println("%");
  } else {
    Serial.println("[BAT] Error enviando estado");
  }
}

void checkBatteryLevel(){
  if(millis() - lastBatteryReport >= BATTERY_REPORT_INTERVAL_MS){
    sendBatteryUpdate();
    lastBatteryReport = millis();
  }
  
  static unsigned long lastCriticalCheck = 0;
  if(millis() - lastCriticalCheck >= 300000) { // 5 minutos
    lastCriticalCheck = millis();
    
    float voltage = readBatteryVoltage();
    int percentage = getBatteryPercentage(voltage);
    
    if(percentage <= BATTERY_CRITICAL_PERCENT && !batteryLowAlertSent){
      Serial.println("Batería crítica");
      
      char body[256];
      if(gps.location.isValid() && gps.date.isValid() && gps.time.isValid()){
        snprintf(body, sizeof(body), 
                 "{\"alert\":\"battery_critical\",\"voltage\":%.3f,\"percentage\":%d,\"lat\":%.6f,\"lon\":%.6f,\"date\":\"%02d/%02d/%04d\",\"time\":\"%02d:%02d:%02d\"}", 
                 voltage, percentage, gps.location.lat(), gps.location.lng(),
                 gps.date.day(), gps.date.month(), gps.date.year(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
      } else {
        snprintf(body, sizeof(body), 
                 "{\"alert\":\"battery_critical\",\"voltage\":%.3f,\"percentage\":%d}", 
                 voltage, percentage);
      }
      
      String url = fbUrl(fbPathAlerts());
      if(simHttpPost(url.c_str(), body)) {
        Serial.println("Alerta crítica enviada");
        batteryLowAlertSent = true;
        buzzerStart(100); 
        buzzerStart(100); 
        buzzerStart(100); 
      }
    }
    
    if(percentage > BATTERY_CRITICAL_PERCENT + 5) {
      batteryLowAlertSent = false;
    }
  }
}

// ================= Aplicación =================
void sendAlert(float gmag){
  char body[256];
  if(gps.location.isValid() && gps.date.isValid() && gps.time.isValid()){
    snprintf(body,sizeof(body),"{\"alert\":\"fall\",\"magnitude\":%.2f,\"lat\":%.6f,\"lon\":%.6f,\"date\":\"%02d/%02d/%04d\",\"time\":\"%02d:%02d:%02d\"}", gmag,gps.location.lat(),gps.location.lng(), gps.date.day(),gps.date.month(),gps.date.year(), gps.time.hour(),gps.time.minute(),gps.time.second());
  } else snprintf(body,sizeof(body),"{\"alert\":\"fall\",\"magnitude\":%.2f}", gmag);
  String url = fbUrl(fbPathAlerts());
  if(!simHttpPost(url.c_str(), body)) Serial.println("Error envio alerta"); else Serial.println("Alerta enviada");
}

void sendTrackingUpdate(){
  char body[256];
  if(gps.location.isValid() && gps.date.isValid() && gps.time.isValid()){
    snprintf(body,sizeof(body),"{\"type\":\"track\",\"lat\":%.6f,\"lon\":%.6f,\"date\":\"%02d/%02d/%04d\",\"time\":\"%02d:%02d:%02d\"}", gps.location.lat(),gps.location.lng(), gps.date.day(),gps.date.month(),gps.date.year(), gps.time.hour(),gps.time.minute(),gps.time.second());
  } else snprintf(body,sizeof(body),"{\"type\":\"track\",\"status\":\"no_fix\"}");
  String url = fbUrl(fbPathTracks());
  if(simHttpPost(url.c_str(), body)) Serial.println("Tracking enviado"); else Serial.println("Tracking fallo HTTP");
}

void pollRemoteCommand(){
  if(millis()-lastCommandPoll < COMMAND_POLL_INTERVAL_MS) return; lastCommandPoll=millis();
  String url = fbUrl(fbPathCmd());
  String body; 
  
  if(!simHttpGet(url.c_str(), body)){ 
    Serial.println("Cmd: fallo GET"); 
    return; 
  }
  
  Serial.print("Cmd: respuesta Firebase: '"); Serial.print(body); Serial.println("'");
  if(body == "null" || body.length() == 0) {
    Serial.println("Cmd: sin comandos pendientes (null)");
    return;
  }
  
  String up=body; up.toUpperCase();
  if(up.indexOf("\"TRACK\":TRUE")!=-1){ 
    if(!trackingActive){ 
      trackingActive=true; 
      lastTrackSent=0; 
      Serial.println("CMD: TRACK ON"); 
    }
  }
  else if(up.indexOf("\"TRACK\":FALSE")!=-1){ 
    if(trackingActive){ 
      trackingActive=false; 
      Serial.println("CMD: TRACK OFF"); 
    }
  }
}

// ================= Setup / Loop =================
void setup(){
  Serial.begin(115200); while(!Serial){}
  led.begin(); led.setBrightness(60); setColor(8,0,0);
  gpsSerial.begin(9600); delay(150);
  ModemSerial.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(200);
  Serial.print("[MODEM] UART fijo RX="); Serial.print(MODEM_RX_PIN); Serial.print(" TX="); Serial.print(MODEM_TX_PIN); Serial.println(" @115200");
  Serial.println("Dispositivo iniciado (A7670SA+TinyGSM)");
  pinMode(BUZZER_PIN,OUTPUT); ledcSetup(buzzerChannel, BUZZER_FREQ, 10); ledcAttachPin(BUZZER_PIN,buzzerChannel); ledcWrite(buzzerChannel,0);
  
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12); 
  Serial.println("ADC configurado para monitoreo de batería");
  
  delay(1000);
  float initialVoltage = readBatteryVoltage();
  int initialPercentage = getBatteryPercentage(initialVoltage);
  Serial.print("Batería inicial: "); Serial.print(initialVoltage, 3); 
  Serial.print("V ("); Serial.print(initialPercentage); Serial.println("%)");
  
  mpuOk=initMPU(); Serial.println(mpuOk?"MPU iniciado":"MPU no detectado");
  
  if(initModem()) {
    Serial.println("[MODEM] TinyGSM conectado");
    setColor(0,8,0); // Verde = conectado
    
    sendBatteryUpdate();
    lastBatteryReport = millis();
  } else {
    Serial.println("[MODEM] TinyGSM fallo");
    setColor(8,0,0); // Rojo = error
  }
}

void loop(){
  while(gpsSerial.available()){ uint8_t c=gpsSerial.read(); lastByteMs=millis(); gps.encode(c);}  

  updateLED();
  handleMPU();
  buzzerUpdate();
  pollRemoteCommand();
  checkNetwork(); 
  checkBatteryLevel(); 

  if(trackingActive && millis()-lastTrackSent >= TRACK_INTERVAL_MS){ sendTrackingUpdate(); lastTrackSent=millis(); }

#if ENABLE_AT_BRIDGE
  if(Serial.available()){
    String line=Serial.readStringUntil('\n'); line.trim();
    if(line.startsWith("!AT")){
      String cmd=line.substring(1); Serial.print("[BRIDGE>>] "); Serial.println(cmd);
      Serial.println("[BRIDGE] No disponible con TinyGSM");
    } else if(line.equalsIgnoreCase("!UART")){
      Serial.print("[UART] RX="); Serial.print(MODEM_RX_PIN); Serial.print(" TX="); Serial.print(MODEM_TX_PIN); Serial.print(" Baud="); Serial.println(115200);
    }
  }
#endif

  if(millis()-lastPrint>=1000){
    if(gps.location.isValid()){
      Serial.print(goodFix()?"GOOD ":"FIX  ");
      Serial.print("Lat:"); Serial.print(gps.location.lat(),6); Serial.print(" Lon:"); Serial.print(gps.location.lng(),6);
      if(gps.satellites.isValid()) { Serial.print(" Sats:"); Serial.print(gps.satellites.value()); }
      if(gps.hdop.isValid()) { Serial.print(" HDOP:"); Serial.print(gps.hdop.value()/100.0,1); }
      if(gps.altitude.isValid()) { Serial.print(" Alt:"); Serial.print(gps.altitude.meters(),1); }
      Serial.println();
    } else if(gps.satellites.isValid()) Serial.print("Sin fix. Sats:"), Serial.println(gps.satellites.value());
    else if(millis()-lastByteMs < 3000) Serial.println("Esperando sats...");
    else Serial.println("Sin datos GPS");
    
    // Mostrar estado de batería cada 10 segundos
    static int printCounter = 0;
    if(++printCounter >= 10) {
      printCounter = 0;
      float voltage = readBatteryVoltage();
      int percentage = getBatteryPercentage(voltage);
      Serial.print("[BAT] "); Serial.print(voltage, 3); Serial.print("V ("); 
      Serial.print(percentage); Serial.print("%) Red:"); 
      Serial.print(networkConnected ? "OK" : "NO");
      Serial.print(" NextReport:"); Serial.print((BATTERY_REPORT_INTERVAL_MS - (millis() - lastBatteryReport))/60000);
      Serial.println("min");
    }
    
    lastPrint=millis();
  }
}