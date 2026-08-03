#include "imu.h"
#include "Timer.h"
#include "ubx.h"
#include <map>
#include <Servo.h>
#include <PWMServo.h>
using namespace std;

struct geoPoint {
    double lat;
    double lon; //in degrees
  };

class control {
  private:
  //creating objects for every class in this code, so I can interfact with each sensor & the various stuff I think is necessary to have global
    //allows the main code to 'own' the actual objects, and pass it over to manipulate it here
    //allows the definition of pins etc once: in the main code
    IMUDriver mImu;
    Timer mTimer; //want a global timer that keeps track of how long everything is taking (we need to ensure that the expedition lasts for a fixed period of time)
    bfs::Ubx mGNSS;
    Servo mLeftMotor;
    Servo mRightMotor;
    PWMServo mPumpMotor;

    double northCorrection = mImu.data.heading;
    double desiredBearing = 0.0;

  //map for storing waypoints
    std::map<int, geoPoint> waypoints{ 
      {1, {53.946551, -1.027365}},
      {2, {53.946208, -1.027300}},
      {3, {53.946018, -1.027054}},
      {4, {53.946187, -1.026323}},
      {5, {53.946369, -1.025681}},
      {6, {53.946655, -1.025039}},
      {7, {53.946966, -1.024280}},
      {8, {53.947256, -1.024774}},
      };
    //waypoint data here. pick gnss coords and populate map
    unsigned int currentWaypointIndex = 1;
    geoPoint *currentWaypoint = & waypoints[1]; //use -> when getting waypoint gnss data

    int powerdiff = 0;
    int throttle = 0;
    double lastYaw = 0;
    double integrateYaw(); //technically doesn't exist here for how I want the code to work
    bool waypointsLeft = 1;
    void updateCurrentWaypoint();
    bool positionError(double errorVal);
    double angleError = 0.0;
    void calcAngleError();

    //pump stuff
    void collection_cycle();
    bool out_of_range(float ph, float temp);
    float averageph(int samples);
    float averagetemperature(int samples);
    float readtemperature();

    const int RELAY = 6;
    unsigned long t;
    int cycles = 0;
    int step = 0;

  public:
    control(IMUDriver& mImu_, Timer& mT_, bfs::Ubx& mGNSS_, Servo& mLeftMotor_, Servo& mRightMotor_, PWMservo& mPumpMotor_) : mImu(mImu_), mTimer(mT_), mGNSS(mGNSS_), mLeftMotor(mLeftMotor_), mRightMotor(mRightMotor_), mPumpMotor(mPumpMotor_) {} 
    //double getAngleError();
    bool getWaypointsLeft();
    void returnToBase();
    bool navigateToWaypoints(int throttle);
    
};
//read temp fn coded by Jasper Gaitley
float control::readtemperature() 
{
    sensors.requestTemperatures(); 
    float tempC = sensors.getTempCByIndex(0);
    return tempC;
}

//temp averages over 30s, coded by Jasper Gaitley
float control::averagetemperature(int samples)
{
    float total = 0;
    for(int i = 0; i<samples; i++)
    {
        total += readtemperature();
        delay(50);
    }
    return total/samples;
}
//ph averages over 30s
float control::averageph(int samples)
{
    float total = 0;
    for(int i = 0; i<samples; i++)
    {
        voltage = analogRead(PH_PIN)/1024.0*3300;  // read the voltage
        float temp = readtemperature();
        total += ph.readPH(voltage, temp);
        delay(50);
    }
    return total/samples;
}

//function for range of ph and temp coded by Jasper Gaitley
bool control::out_of_range(float ph, float temp)
{
    bool ph_flag = (ph < pH_min || ph > pH_max);
    bool temp_flag = (temp < temperature_min || temp > temperature_max);
    return (ph_flag || temp_flag);
}

//function including all the steps of water collection

void control::collection_cycle() //coded by Jasper Gaitley
{
    now = millis();
    if (step == 0 && now - t >= 0) {          // pump ON
    digitalWrite(RELAY, HIGH);
    t = now;
    step = 1;
    }
   
    else if (step ==1 && now - t >= 775) { 
        digitalWrite(RELAY, LOW); 
        fs90r.write(95);
        t = now; 
        step = 2; 
    }

    else if (step == 2 && now - t >= 310) {   // servo run
    fs90r.write(90);
    t = now;
    step = 3;
    }

    else if (step == 3 && now - t >= 500) {   // servo stop
    cycles++;
    t = now;
    step = 0;
    }
}
void control::returnToBase() {
  currentWaypoint = & waypoints[8];
}

void control::updateCurrentWaypoint(){ //fn to update current way point
  if (currentWaypointIndex<waypoints.size()) {//placeholder no. adjust when adam sends over waypoints
    currentWaypoint = & waypoints[currentWaypointIndex];
  }
  else {
    waypointsLeft = 0;
  }
}
bool control::navigateToWaypoints(int throttle) {
  calcAngleError();
  if (positionError(1.0)) { //returns a 1 if there is a position error
      if (angleError>10.0 || angleError<-10.0) { 
        if (angleError<0) {
          powerdiff = -100;
        }
        else {
          powerdiff = 100; //dunno what these values r gonna do
        }
      }
      else {
        powerdiff = 0;
      }
      mLeftMotor.writeMicroseconds(throttle + powerdiff);
      mRightMotor.writeMicroseconds(throttle);
      return 0;
    }
  else {
    return 1;
    updateCurrentWaypoint();
    static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U){ // the rest of this function is coded by Jasper Gaitley. Ideally, this would be refined but due to its lack of use within the final system, this was considered unncessary within the scope of the project
      timepoint = millis();
      
      phValue = averageph(sample_number);  
      temperature = averagetemperature(sample_number);        
      Serial.print("temperature:");
      Serial.print(temperature);
      Serial.print("^C  pH:");
      Serial.println(phValue,2);
  }

  voltage = analogRead(PH_PIN)/1024.0*3300;
  ph.calibration(voltage,temperature);           // calibration process by Serail CMD
  
  
  //STOPS SERVO AND PUMP IF ALL SAMPLES FILLED OR IF RANGES ARE NORMAL
  if (cycles >= 4 || !out_of_range(phValue, temperature))
  { 
      fs90r.write(90);
      digitalWrite(RELAY, LOW); 
      return; 
  }

  if (out_of_range(phValue,temperature) &&cycles <4)
  {
      collection_cycle();
  }
  }
} 

bool control::getWaypointsLeft(){
  return waypointsLeft;
}

double control::integrateYaw(){ //using trapezium method. test computationally
  uint32_t t1 = mTimer.read();
  for (int i=0; i<10; i++) {
    double currentYaw = mImu.data.angle.z;
    uint32_t t2 = mTimer.read();
    lastYaw = (currentYaw + lastYaw)*(t2 - t1);
    t1 = t2;
  }
  return lastYaw;
} //to be placed into IMU class somewhere as a public method.

void control::calcAngleError() {
  //getting the 'bearing' angle
  double bearingActual = integrateYaw() - northCorrection;
  while (bearingActual > 360.0) { //dont think this will happen tbh
    bearingActual = bearingActual - 360.0; 
  }
  //bearingActual = bearingActual - mImu.getNorthCorrection(); //imaginary function. dunno how it works rn
  if (bearingActual<0.0) {
    bearingActual = 360.0 + bearingActual;
  }

  double vectorDesired[2] = {(currentWaypoint->lat - mGNSS.lat_deg()), (currentWaypoint->lon - mGNSS.lon_deg())};
  double vectorAngle = atan(abs(vectorDesired[1]/vectorDesired[0])); //in radians by default i think
  
  //getting angle in a bearing format:
  if (vectorDesired[0]==0.0) {
    if (vectorDesired[1]>0) {
      desiredBearing = 0.0;
    }
    else if (vectorDesired[1]<0) {
      desiredBearing = 180.0;
    }
  }
  else if (vectorDesired[0]>0.0) {
    if (vectorDesired[1]>0.0) {
      desiredBearing = 90.0 - vectorAngle;
    }
    else if (vectorDesired[1]<0.0) {
      desiredBearing = 90.0 + vectorAngle;
    }
    else {
      desiredBearing = 90.0;
    }
  }
  else if (vectorDesired[0]<0.0) {
    if (vectorDesired[1]>0.0) {
      desiredBearing = 270.0 + vectorAngle;
    }
    else if (vectorDesired[1]<0.0) {
      desiredBearing = 270.0 - vectorAngle;
    }
    else {
      desiredBearing = 270.0;
    }
  }

  angleError = desiredBearing - bearingActual;
}

bool control::positionError(double errorValue) {
  if (abs(mGNSS.lat_deg()-currentWaypoint->lat)<errorValue){
    if (abs(mGNSS.lon_deg() - currentWaypoint->lon)<errorValue) {
      return 0;
    }
  }
  return 1;
}