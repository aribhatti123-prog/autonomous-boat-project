#include "ubx.h" //gnss sensor
#include "lora.h"

//imu
#include "imu.h"
IMUDriver imu;

//temp
#include "string.h"
#include "DallasTemperature.h"
#define ONE_WIRE_BUS A5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

//pH
#include "DFRobot_PH.h"
#include <EEPROM.h>

#define PH_PIN A2
double voltage,temp;
DFRobot_PH phSensor;

//pump
#include <PWMServo.h>
PWMServo pumpMotor;
const int RELAY = 6;
unsigned long t;
unsigned long now = millis();
int cycles = 0;
int step = 0;

Timer timer(MILLIS);
bfs::Ubx gnss(&Serial1); 
lora lora(Serial7, timer);

void setup() {
  // put your setup code here, to run once:
  imu.begin();
  gnss.Begin(38400);
  gnss.fix();
  Serial7.begin(9600); //lora
  phSensor.begin();
  tempSensor.begin();
  pumpMotor.attach(5);
  pinMode(RELAY, OUTPUT);
  timer.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(24000);
  float temp = 0.0;
  float ph = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double heading = 0.0;
  bool atWaypoint = 0;
  bool waterCollectionSystem = 0;
  for (int i=0; i<24; i++) {
    uint32_t lastSample = timer.read();
    float samples = 0.0;
    float tempTotal = 0;
    float phTotal = 0;
    while ((timer.read()-lastSample)<10000) { //samples our temp n pH over 10s
      tempTotal = readtemperature() + tempTotal;
      phTotal = readph() + phTotal;
      samples = samples+1;
    }
    if (i==12) {
      waterCollectionSystem = 1;
      atWaypoint = 1;
      for (int i = 0; i<2; i++) {
        collection_cycle();
      }
    }
    else if (i%3 == 0) {
      atWaypoint = 1;
    }
    else {
      atWaypoint = 0;
    }
    temp = tempTotal/samples;
    ph = phTotal/samples;
    lat = gnss.lat_deg();
    lon = gnss.lon_deg();
    heading = imu.data.heading;
    lora.updateDataToSend(temp, ph, lat, lon, heading, atWaypoint, waterCollectionSystem);
    lora.transmitData();
  }
}

//read temp fn
float readtemperature() 
{
    tempSensor.requestTemperatures(); 
    float tempC = tempSensor.getTempCByIndex(0);
    return tempC;
}

//read pH
float readph() {
  float voltage = analogRead(PH_PIN)/1024.0*3300;
  float temp = readtemperature();
  return phSensor.readPH(voltage, temp);
}

void collection_cycle()
{
    now = millis();
    if (step == 0 && now - t >= 0) {          // pump ON
    digitalWrite(RELAY, HIGH);
    t = now;
    step = 1;
    }
   
    else if (step ==1 && now - t >= 750) { 
        digitalWrite(RELAY, LOW); 
        pumpMotor.write(96);
        t = now; 
        step = 2; 
    }

    else if (step == 2 && now - t >= 360) {   // servo run
    pumpMotor.write(90);
    t = now;
    step = 3;
    }

    else if (step == 3 && now - t >= 500) {   // servo stop
    cycles++;
    t = now;
    step = 0;
    }
}