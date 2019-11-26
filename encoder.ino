#include <ArduinoJson.h>
#include <RobotLib.h>

const byte interruptPinA = 2;
const byte interruptPinB = 3;
volatile byte state = LOW;

long ticks;

QuadratureEncoder speedyboi();
speedyboi.begin(2,3,1);

void getTicks(){
  
  speedyboi.process();
  
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println();
  
  // put your setup code here, to run once:
  pinMode(interruptPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), getTicks, CHANGE);

  Serial.println("Start: ");
  char JSONMessage[] = "{\"ticks\": 0  }"; //Original Message
  StaticJsonBuffer<200> JSONBuffer; //Memory Pool
  JsonObject& parsedMessage = JSONBuffer.parseObject(JSONMessage);

  //This should never happen. Used for bug testing JSON Message Parsing.
  if (!parsed.success()){
    Serial.println("Parsing Message Failed!");
    delay(1000);
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  ticks = speedyboi.getTicks();
  speedyboi.reset();
  //send ticks to raspberry pi
  doc["ticks"] = ticks;
  serializeJson(doc, Serial);
  Serial.println();
  doc["ticks"] = 0;
}
