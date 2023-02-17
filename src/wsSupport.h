void wsloop();
void wsSetup();
#include <WString.h>

extern String getValue(String data, char separator, int index);
extern String getIO();
extern String setIO(int pin,int status);
extern String setPWM(int pin,int freq,int res,int duty);
//extern String getTemperature();
extern void wssend(String wsm);