#include <RH_ASK.h>
#include <SPI.h> 

RH_ASK driver(2000, 11, 12);

uint8_t joystick[2];

void setup()
{
    Serial.begin(9600);    
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    // read values from x and y pot of joystick
    int xVal = analogRead(A0);
    int yVal = analogRead(A1);

    // map them to 8bit integers for transfering over RF
    uint8_t x_map = map(xVal, 0, 1023, 0, 255);
    uint8_t y_map = map(yVal, 0, 1023, 0, 255);

    // add values to joystick array
    joystick[0] = x_map;
    joystick[1] = y_map;
    
    // send array
    driver.send((uint8_t *)j, sizeof(joystick));
    driver.waitPacketSent();
    delay(200);
}
