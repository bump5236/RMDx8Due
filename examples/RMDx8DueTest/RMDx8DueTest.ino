#include <RMDx8Due.h>    //librariesに入っていないと読み込めない


#define BAUDRATE 115200

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1~32)

RMDx8Due rmd1(CAN0, MOTOR_ADDRESS);

void setup() {
    Serial.begin(BAUDRATE);
    delay(1000);

    rmd1.canSetup();

    rmd1.readPID();

    Serial.print("POSKp1:");
    Serial.println(rmd1.posKp);
    Serial.print("POSKi1:");
    Serial.println(rmd1.posKi);
    delay(3000);
}

void loop() {
    rmd1.writeCurrent(0);
    rmd1.readPosition();

    Serial.print("CUR1:");
    Serial.print(rmd1.present_current);
    Serial.print("\t");
    Serial.print("VEL1:");
    Serial.print(rmd1.present_velocity);
    Serial.print("\t");
    Serial.print("POS1:");
    Serial.print(rmd1.present_position);
    Serial.print("\t");
}