#include "control.h"
#include "ubx.h"
#include "lora.h"

//temp
#include "string.h"
#include "DallasTemperature.h"
#define ONE_WIRE_BUS A5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
float temperature = 25.00;

//ph
#include "DFRobot_PH.h"
#include <EEPROM.h>
float voltage,phValue;

#define PH_PIN A2
DFRobot_PH phSensor;

//pump
#include <PWMServo.h>
PWMServo fs90r;

//temp and ph range setup
const float pH_min = 6.5;
const float pH_max = 8.0;

const float temperature_min = 10;
const float temperature_max = 30;

const int sample_number = 30;

Timer timer(MICROS);
IMUDriver imu; //hard coded to be on serial 2. ideally change to be passed the correct pin
bfs::Ubx gnss(&Serial3); //double check syntax
Servo esc1; //left
Servo esc2; //right
control controller(imu, timer, gnss, esc1, esc2);
lora lora(Serial7, timer);

int throttle = 1500; //neutral
bool atWaypoint = 0;


void setup() {
  // put your setup code here, to run once:
  imu.begin();
  gnss.Begin(38400);
  gnss.fix();
  //motors setup
  esc1.attach(2);
  esc2.attach(3);
  esc1.writeMicroseconds(1500);
  esc2.writeMicroseconds(1500);
  delay(2000);
  Serial7.begin(9600);
  ph.begin();
  sensors.begin();
  fs90r.attach(5);
  pinMode(RELAY, OUTPUT);
  t = millis();
  timer.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  while (controller.getWaypointsLeft() && timer.read()<100) { //returns a 1 if not at the final waypoint TIMER VALUE IS A PLACEHOLDER
    atWaypoint = controller.navigateToWaypoints(throttle);
    updateDataToSend(atWaypoint);
    lora.transmitData();
  }
  controller.returnToBase();
  atWaypoint = controller.navigateToWaypoints(throttle);
  updateDataToSend(atWaypoint);
  lora.transmitData();


}

void updateDataToSend(bool atWaypoint) {
  float temp = controller.requesttemperature();
  float ph = getpH();
  double lat = gnss.lat_deg();
  double lon = gnss.lon_deg();
  lora.updateDataToSend(temp, ph, lat, lon, atWaypoint);
}

float getpH() {
  float voltage = analogRead(PH_PIN)/1024.0*3300;
  float temp = getTemp();
  return ph.readPH(voltage,temp);
}