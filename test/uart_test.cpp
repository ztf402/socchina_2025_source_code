#include <Arduino.h>
#include <HardwareSerial.h>
HardwareSerial Serial5(UART5);

void setup(void){
    Serial5.setRx(PD_2);
    Serial5.setTx(PC_12);
    Serial5.begin(115200);
}
void loop(void){
    Serial5.println("AT");
    delay(400);
}