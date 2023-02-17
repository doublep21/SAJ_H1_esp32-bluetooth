/* https://github.com/gilmaimon/ArduinoWebsockets */
#include <ArduinoWebsockets.h>
#include <WiFi.h>

const char* websockets_server = "ws://192.168.0.4:6080"; //server adress and port
unsigned int ConnectionTimeout = 5000;

extern String getValue(String data, char separator, int index);
extern String getIO();
extern String setIO(int pin,int status);
//extern String getTemperature();
extern String setPWM(int pin,int freq,int res,int duty);

bool cc = false;
void connectws();

using namespace websockets;
WebsocketsClient client;


void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
    String data = message.data();
    if (data.charAt(0) == '/') {
        // if(data == "/gettemp"){
        //     client.send(getTemperature());
        // }
        /*
        if(data == "/getio"){
            client.send("/io?object="+getIO());
        }*/
        if(getValue(data, '?', 0) == "/setio"){
            if(getValue(getValue(getValue(data, '?', 1), '&', 0), '=', 0) == "pin"){
                String pin = getValue(getValue(getValue(data, '?', 1), '&', 0), '=', 1);
                String status = getValue(getValue(getValue(data, '?', 1), '&', 1), '=', 1);        
                client.send(setIO( pin.toInt() , status.toInt() )); 
            }      
        }
        if(getValue(data, '?', 0) == "/setpwm"){
            if(getValue(getValue(getValue(data, '?', 1), '&', 0), '=', 0) == "pin"){
                String pin = getValue(getValue(getValue(data, '?', 1), '&', 0), '=', 1);
                String freq = getValue(getValue(getValue(data, '?', 1), '&', 1), '=', 1);
                String res = getValue(getValue(getValue(data, '?', 1), '&', 2), '=', 1); 
                String duty = getValue(getValue(getValue(data, '?', 1), '&', 3), '=', 1);         
                client.send(setPWM( pin.toInt() , freq.toInt(), res.toInt(), duty.toInt() )); 
            }      
        }
    }
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connnection Opened");
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connnection Closed");
    } else if(event == WebsocketsEvent::GotPing) {
        Serial.println("Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        Serial.print(millis());
        Serial.println(": Got a Pong!");
        cc = true;
    }
}


void wsSetup(){
     // Setup Callbacks
    //client.setInsecure();
    client.onMessage(onMessageCallback);
    client.onEvent(onEventsCallback);
    connectws();
}


void connectws(){
    // Connect to server
    // Before connecting, set the ssl fingerprint of the server
    // client.setCACert(ssl_ca_cert);
    bool connected = client.connect(websockets_server);
    if(connected) {
        Serial.println("Connected!");
        client.send("/login?ESP="+WiFi.macAddress()+"&auth="+"00000000");
        //client.send("WIFI RSSI:"+String(WiFi.RSSI()));
        cc = true;
    } else {
        cc = false;
        Serial.println("Not Connected!");
    }
}

void connectionCheck(){
    //client.send("ping");
    client.ping();
    cc = false;
}

extern void wssend(String wsm){
    client.send(wsm);
}

unsigned long startmillis = millis();
String s;
void wsloop(){
    if (startmillis + ConnectionTimeout < millis()){
        startmillis = millis() + ConnectionTimeout;
        if(!cc){
            connectws();
        }
        connectionCheck();
    }
    client.poll();
    if (Serial.available() > 0) {
        s = Serial.readString();
        client.send(s);
    }
}