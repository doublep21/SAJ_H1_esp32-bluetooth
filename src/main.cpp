#include <wsSupport.h>
#include <wifiSettings.h>
#include <wifi.h>
#include <ArduinoOTAC.h>
#include "BLEDevice.h"

#define logln(a) {Serial.println(a);wssend((String) a);}
#define log(a) {Serial.print(a);wssend((String) a);}

unsigned long previousMillis = 0;
unsigned long previousMillistry = 0;

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// #define FORMAT_SPIFFS_IF_FAILED true

static BLEUUID serviceUUID("0000ffff-0000-1000-8000-00805f9b34fb");
static BLEUUID    charUUIDw("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID    charUUID("0000ff02-0000-1000-8000-00805f9b34fb");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicw;
static BLEAdvertisedDevice* myDevice;
extern bool otaupdating;
BLEClient*  pClient  = BLEDevice::createClient();
bool delaytry = false;
bool errorr = false;
int reg = 0;
bool waitingmessage = false;
int c = 0;
int soft = 0;
extern void wssend(String wsm);



void ota_handle( void * parameter ) {
  
  for (;;) {
    
    wifiloop();
    otaloop();
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
void wstask( void * parameter ) {
  
  for (;;) {
    
     if (WiFi.status() == WL_CONNECTED){
         wsloop();
    }
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}


extern String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

extern String setIO(int pin,int status){
    pinMode(pin, OUTPUT);
    if(status==2){
        int ps = digitalRead(pin);
        if(ps == 1){
            digitalWrite(pin,0);
            status = 0;
        }else{
            digitalWrite(pin,1);
            status = 1;
        }
    }else{
        digitalWrite(pin,status);
    }
    return "/buttonchange?pin="+String(pin)+"&status="+String(status);
}

extern String setPWM(int pin,int freq,int res,int duty){
    ledcSetup(pin, freq, res);
    ledcAttachPin(pin, pin);
    ledcWrite(pin, duty);
    return "/pwmchange?pin="+String(pin)+"&duty="+String(duty);
}

String myuint16(unsigned char byte1, unsigned char byte2, float scale) {
    
    unsigned int combined = (byte1 << 8) | byte2;
    
    float result = (float)combined*scale;
    
    
    return String(result);
}
String myint16(unsigned char byte1, unsigned char byte2, float scale) {
    
    signed int combined = (short)((byte1 << 8) | byte2);
    
    float result = (float)combined*scale;
    
    return String(result);
}
String myuint32(unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4, float scale) {
    
    unsigned long combined = (unsigned long)((byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4);
    
    float result = (float)combined * scale;
    
    return String(result);
}


int errlen = 0;
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    waitingmessage = false;
    if((int)length < 25){
      errlen = errlen +1;
      String sdata = "ERROR BYTE [";
      for(int i=0; i<length; i++){
        //int temp = pData[i];
        //log((int8_t) pData[i]);
        sdata = sdata + (String)(int8_t) pData[i]+",";
      }
      sdata = sdata + "]";
      wssend(sdata);
      if(errlen > 10){
        pClient->disconnect();
        errorr = true;
        log("sending disconnect signal len="+String(length)+"  - ");
      }else{
        previousMillistry = millis();
      }
    }else{
      errlen=0;
      if(reg == 1){
        previousMillistry = millis();
        String jsonString = "{\"bvoltage\": " + myuint16(pData[8],pData[9],0.1) + "," +
                        "\"BatCurr\": " + myint16(pData[10],pData[11],0.01) + "," +
                        "\"BatCurr1\": " + myint16(pData[12],pData[13],0.01) + "," +
                        "\"BatCurr2\": " + myint16(pData[14],pData[15],0.01) + "," +
                        "\"BatPower\": " + myint16(pData[16],pData[17],1) + "," +
                        "\"BatTempC\": " + myint16(pData[18],pData[19],0.1) + "," +
                        "\"Bat%\": " +  myuint16(pData[20],pData[21],0.01) + "," +
                        "\"pv1volt\": " + myuint16(pData[24],pData[25],0.1) + "," +
                        "\"pv1curr\": " + myuint16(pData[26],pData[27],0.01) + "," +
                        "\"pv1power\": " + myuint16(pData[28],pData[29],1) + "," +
                        "\"pv2volt\": " + myuint16(pData[30],pData[31],0.1) + "," +
                        "\"pv2curr\": " + myuint16(pData[32],pData[33],0.01) + "," +
                        "\"pv2power\": " + myuint16(pData[34],pData[35],1) + "}";
        // if(myuint16(pData[8],pData[9],0.1).toInt(> 56) < 56){
          wssend("/saj?data="+jsonString);
        // }
      }
      if(reg == 0){
        previousMillistry = millis();
        // if(myuint16(pData[22],pData[23],1).toInt()>7000){
        //   // errorr = true;
        //   // log("sending disconnect signal error");
        //   errlen = errlen +1;
        // }
        String jsonString = "{\"SysTotalLoadWatt\": " + myint16(pData[8],pData[9],1) + "," +
                          "\"TotalPVPower\": " + myint16(pData[10],pData[11],1) + "," +
                          "\"TotalBatteryPower\": " + myint16(pData[12],pData[13],1) + "," +
                          "\"TotalGridPowerWatt\": " + myint16(pData[14],pData[15],1) + "," +
                          "\"TotalGridPowerVA\": " + myint16(pData[16],pData[17],1) + "," +
                          "\"TotalInvPowerWatt\": " + myint16(pData[18],pData[19],1) + "," +
                          "\"TotalInvPowerVA\": " +myint16(pData[20],pData[21],1) + "," +
                          "\"BackupTotalLoadPowerWatt\": " + myint16(pData[22],pData[23],1) + "," +
                          "\"BackupTotalLoadPowerVA\": " + myuint16(pData[24],pData[25],1) + "}";
        wssend("/saj?data="+jsonString);//"/saj?data="+
      }
      if(reg == -1){
        previousMillistry = millis();
       String jsonString = "{\"Today_PVEnergy\": " + myuint32(pData[8],pData[9],pData[10],pData[11],0.01) + "," +
                        "\"Month_PVEnergy\": " + myuint32(pData[12],pData[13],pData[14],pData[15],0.01) + "," +
                        "\"Year_PVEnergy\": " + myuint32(pData[16],pData[17],pData[18],pData[19],0.01) + "," +
                        "\"Total_PVEnergy\": " + myuint32(pData[20],pData[21],pData[22],pData[23],0.01) + "," +
                        "\"Today_BatChgEnergy\": " + myuint32(pData[24],pData[25],pData[26],pData[27],0.01) + "," +
                        "\"Month_BatChgEnergy\": " + myuint32(pData[28],pData[29],pData[30],pData[31],0.01) + "," +
                        "\"Year_BatChgEnergy\": " + myuint32(pData[32],pData[33],pData[34],pData[35],0.01) + "," +
                        "\"Total_BatChgEnergy\": " + myuint32(pData[36],pData[37],pData[38],pData[39],0.01) + "," +
                        "\"Today_BatDisEnergy\": " + myuint32(pData[40],pData[41],pData[42],pData[43],0.01) + "," +
                        "\"Month_BatDisEnergy\": " + myuint32(pData[44],pData[45],pData[46],pData[47],0.01) + "," +
                        "\"Year_BatDisEnergy\": " + myuint32(pData[48],pData[49],pData[50],pData[51],0.01) + "," +
                        "\"Total_BatDisEnergy\": " + myuint32(pData[52],pData[53],pData[54],pData[55],0.01) + "," +
                        "\"Today_TotalLoadEnergy\": " + myuint32(pData[72],pData[73],pData[74],pData[75],0.01) + "," +
                        "\"Month_TotalLoadEnergy\": " + myuint32(pData[76],pData[77],pData[78],pData[79],0.01) + "," +
                        "\"Year_TotalLoadEnergy\": " + myuint32(pData[80],pData[81],pData[82],pData[83],0.01) + "," +
                        "\"Total_TotalLoadEnergy\": " + myuint32(pData[84],pData[85],pData[86],pData[87],0.01) + "," +
                        "\"Today_BackupLoadEnergy\": " + myuint32(pData[88],pData[89],pData[90],pData[91],0.01) + "," +
                        "\"Month_BackupLoadEnergy\": " + myuint32(pData[92],pData[93],pData[94],pData[95],0.01) + "," +
                        "\"Year_BackupLoadEnergy\": " + myuint32(pData[96],pData[97],pData[98],pData[99],0.01) + "," +
                        "\"Total_BackupLoadEnergy\": " + myuint32(pData[100],pData[101],pData[102],pData[103],0.01) + "," +
                        "\"Today_SellEnergy\": " + myuint32(pData[104],pData[105],pData[106],pData[107],0.01) + "," +
                        "\"Month_SellEnergy\": " + myuint32(pData[108],pData[109],pData[110],pData[111],0.01) + "," +
                        "\"Year_SellEnergy\": " + myuint32(pData[112],pData[113],pData[114],pData[115],0.01) + "," +
                        "\"Total_SellEnergy\": " + myuint32(pData[116],pData[117],pData[118],pData[119],0.01) + "," +
                        "\"Today_FeedInEnergy\": " + myuint32(pData[120],pData[121],pData[122],pData[123],0.01) + "," +
                        "\"Month_FeedInEnergy\": " + myuint32(pData[124],pData[125],pData[126],pData[127],0.01) + "," +
                        "\"Year_FeedInEnergy\": " + myuint32(pData[128],pData[129],pData[130],pData[131],0.01) + "," +
                        "\"Total_FeedInEnergy\": " + myuint32(pData[132],pData[133],pData[134],pData[135],0.01) + "}" ;

                        // 16623	40EFH	2	Today_SellEnergy	Uint32	-2
                        // 16625	40F1H	2	Month_SellEnergy	Uint32	-2
                        // 16627	40F3H	2	Year_SellEnergy	Uint32	-2
                        // 16629	40F5H	2	Total_SellEnergy	Uint32	-2
                        // 16631	40F7H	2	Today_FeedInEnergy	Uint32	-2
                        // 16633	40F9H	2	Month_FeedInEnergy	Uint32	-2
                        // 16635	40FBH	2	Year_FeedInEnergy	Uint32	-2
                        // 16637	40FDH	2	Total_FeedInEnergy	Uint32	-2    
        
        // wssend(jsonString);
        wssend("/saj?data="+jsonString);
        // String sdata = "[";
        // for(int i=0; i<length; i++){
        //   //int temp = pData[i];
        //   //log((int8_t) pData[i]);
        //   sdata = sdata + (String)(int8_t) pData[i]+",";
        // }
        // sdata = sdata + "]";
        // wssend(sdata);
      }
    }
    
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    
    
    
    
  }
};

bool connectToServer() {
    BLEDevice::getScan()->stop();
    c = 0;
    log("Forming a connection to ");
    connected = true;
    previousMillistry = millis();
    logln(myDevice->getAddress().toString().c_str());
    previousMillistry = millis();
    
    logln(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());
    previousMillistry = millis();
    
    pClient->connect(myDevice);  
    logln(" - Connected to server");
    
    previousMillistry = millis();
    
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      log("Failed to find our service UUID: ");
      logln(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    logln(" - Found our service");

    previousMillistry = millis();
    
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    previousMillistry = millis();
    pRemoteCharacteristicw = pRemoteService->getCharacteristic(charUUIDw);
    if (pRemoteCharacteristic == nullptr) {
      log("Failed to find our characteristic UUID: ");
      logln(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    logln(" - Found our characteristic");

    previousMillistry = millis();
    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    soft = 0;
    return true;
}
/* Scan for BLE servers and find the first one that advertises the service we are looking for.*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /* Called for each advertising BLE server.*/
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // log("BLE Advertised Device found: ");
    // logln(advertisedDevice.toString().c_str());
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    } 
  } 
}; 


void setup() {
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  delay(2000);
  digitalWrite(16, HIGH);
  //digitalWrite(16, HIGH);
  Serial.begin(115200);
  wifisetup();
  otasetup();
  
  xTaskCreate(
      ota_handle,          /* Task function. */
      "OTA_HANDLE",        /* String with name of task. */
      10000,            /* Stack size in bytes. */
      NULL,             /* Parameter passed as input of the task */
      5,                /* Priority of the task. */
      NULL          /* Task handle. */
      );  
  wsSetup();            
  xTaskCreate(
    wstask,          /* Task function. */
    "wstask",        /* String with name of task. */
    5000,            /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    3,                /* Priority of the task. */
    NULL          /* Task handle. */
    );
  
  delay(6000);

  logln("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5000, false);
    
} 

void loop() {
if(!otaupdating){
  if (doConnect == true) {
    if (connectToServer()) {
      logln("We are now connected to the BLE Server.");
      connected = true;
      c = 0;
    } else {
      logln("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
  unsigned long currentMillis = millis();
  // if(!connected){
    if(((currentMillis - previousMillistry) >= 9500)|| errorr){
      doConnect = true;
      connected = false;
      previousMillistry = currentMillis;
      errorr = false;
      log("disconnected no messages - soft");
      soft = soft +1;
      delay(1000);
    }
  // }
  // if(connected){
    if((currentMillis - previousMillistry) >= 9000  ){
      log("disconnected no messages - retrying");
      digitalWrite(16, LOW);
      delay(2000);
      digitalWrite(16, HIGH);
      soft = 0;
      previousMillistry = currentMillis;
      previousMillis = currentMillis + 12000;
      doConnect = true;
      connected = false;
      
    }
  // }

  
 
  if (connected) {
    if(((currentMillis - previousMillis) >= 700) && !waitingmessage){
      previousMillis = currentMillis;
      waitingmessage = true;
      if(c>255){
        c=0;
      }
      if (c % 10 == 0) {reg = 2;}
      if(reg == 0 || reg == -1){byte paylo[13] = {(byte) 77, (byte) 0, (byte) c, (byte) 9, (byte) 50, (byte) 1, (byte) 3, (byte) 64, (byte) 105, (byte) 0, (byte) 14, (byte) 1, (byte) 210};pRemoteCharacteristicw->writeValue(paylo, 13,true);reg=1;}
      else{
        if(reg == 1){byte paylo[13] = {(byte) 77, (byte) 0, (byte) c, (byte) 9, (byte) 50, (byte) 1, (byte) 3, (byte) 64, (byte) 160, (byte) 0, (byte) 14, (byte) 209, (byte) 236};pRemoteCharacteristicw->writeValue(paylo, 13,true);reg=0;}
        else{
          if(reg == 2){byte paylo[13] = {(byte) 77, (byte) 0, (byte) c, (byte) 9, (byte) 50, (byte) 1, (byte) 3, (byte) 64, (byte) 191, (byte) 0, (byte) 62, (byte) 224, (byte) 62};pRemoteCharacteristicw->writeValue(paylo, 13,true);reg=-1;}
          //if(reg == 1){byte paylo[13] = {(byte) 77, (byte) 0, (byte) c, (byte) 9, (byte) 50, (byte) 1, (byte) 3, (byte) 64, (byte) 160, (byte) 0, (byte) 16, (byte) 81, (byte) 228};pRemoteCharacteristicw->writeValue(paylo, 13,true);reg=0;}
        }
        //if(reg == 1){byte paylo[13] = {(byte) 77, (byte) 0, (byte) c, (byte) 9, (byte) 50, (byte) 1, (byte) 3, (byte) 64, (byte) 160, (byte) 0, (byte) 16, (byte) 81, (byte) 228};pRemoteCharacteristicw->writeValue(paylo, 13,true);reg=0;}
      }
      c=c+1;
    }
  }else if(doScan){ 
    logln("scanning again");
    BLEDevice::getScan()->start(0);  
  }

}else{
  digitalWrite(16, LOW);
  //BLEDevice::getScan()->stop();
}
} 