// Tacit, Wrist mounted tactile feedback for the blind.
//   By Steve Hoefer at Grathio Labs (http://grathio.com)
//     Version 12.02.04
//
//  Copyright (c) 2012 Steve Hoefer
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
//  associated documentation files (the "Software"), to deal in the Software without restriction,  
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
//   subject to the following conditions:
//  
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software. 
//   
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT 
//  LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN 
//  NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
//  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//
//  Written for Arduino authoring environment version 0022 and a Arduino Mini Pro 5V but should work on any Arduino/Arduino compatible that provides 5 volts.
//
// This version supports the following hardware:
//      Parallax PING))) ultrasonic sensors for range finding
//               Connect the GND pin to ground, +5V pin to +5V, and SIG to pin 2 or 3.
//      Just about any small hobby servo. (Specifically Turnigy TG9)
//               Connect the ground to ground, +V to RAW and the signal to pins 7 or 8.
//
//
// Version history:
//  11.08.07 - original
//  11.08.14 - Added code so servos apply constant pressure even when readings are not changing.
//  11.10.11 - Added pause after each sensor reading to fix reported issue with left sensor occasionally giving garbage responses.
//  12.02.04 - Changed license from CC BY-NC-SA to the MIT license.

#include <Servo.h>
const int MaxSensors = 2;                     // The number of sensor/servo pairs.
const int ServoPins[MaxSensors] = {7, 8};     // The pins they're on
const int RangingPins[MaxSensors] = {3, 2};   // The pins they're on
const int ReadingsPerSensor = 5;              // The number of historic readings to consider when determining position.
const int TimePerDegree = 9;                  // ms per degree rotation on the servo to prevent servo motor electrical noise from interfering with the ultrasonic sensor readings
const int MinimumTurnDistance = 3;            // Minimum number of degrees that the servo will turn. Keeps the servos from being too twitchy.

// Variables
Servo ServoList[MaxSensors];                         // Array of servo objects for manipulating easily.
int sensorReadings[MaxSensors][ReadingsPerSensor];   // Hold past readings for each sensor.
int calculatedSenorReadings[MaxSensors];             // The calculated distance for each sensor.
int latestReading = 0;                               // Current position in the array for the most recent reading.
int servoLocations[MaxSensors];                      // The current position of each sensor.
int SenorClose = 500;                                // Closest value we detect with the PING sensor. (Soundwave travel time in milliseconds.)
int SensorFar = 14000;                               // Furthest distance we register on the PING sensor. (Soundwave travel time in milliseconds.)
int ServoClose[MaxSensors] = {0, 160};               // Angle the servo turns to when something is closest.
int ServoFar[MaxSensors] = {70,110};                 // Angle the servo turns to when something is at its furthest.

void setup() {

  //Serial.begin(115200);   				// Uncomment the Serial.foo lines for testing.
  //Serial.println("Begin...");

  // Initialize the servo location and move them through a full range of motion so we know they work.
  for (int i = 0; i < MaxSensors; i++){
     ServoList[i].attach(ServoPins[i]);
     delay(10);
     ServoList[i].write(ServoClose[i]);
     delay(500);
     ServoList[i].write(ServoFar[i]);
     delay(500);
     ServoList[i].detach();
   }
   delay(100);


}

void loop(){
  int i, j, oldLocation;
  unsigned long delayTime;

  // Loop through each range sensor
  for (i = 0; i < MaxSensors; i++){
    // Get the current sensor's range.
    sensorReadings[i][latestReading] = getDistance(i);
    // Figure out an averaged/smoothed readings based on this and past data.
    calculatedSenorReadings[i] = calculateNewDistace(i);

    // Set the servo to the correct angle.
    oldLocation = servoLocations[i];
    servoLocations[i] = map(calculatedSenorReadings[i], 0, 100, ServoClose[i], ServoFar[i]);

    if (latestReading >= ReadingsPerSensor-1){                          // Don't do anything until we have enough data to trend.
      if (abs(servoLocations[i]-oldLocation) >= MinimumTurnDistance){   // Only try to turn it if we have somewhere to go.
		  ServoList[i].attach(ServoPins[i]);
		  delay(10);
		  ServoList[i].write(servoLocations[i]);
		  delayTime = (TimePerDegree * (abs(servoLocations[i]-oldLocation))+20);      // Set a delay for the next reading so motor noise doesn't interfere with senor readings.
		  if (abs(delayTime)>500){ // If it can't do it in this amount of time       // It's based on how far it has to turn to keep the delay to a minimum, response time at a maximum.
			delayTime=500;         // we'll get it next time. Keep it responsive.
		  }
		  delay(delayTime);
		  ServoList[i].detach();
	  } else {                                          // Otherwise if the reading hasn't changed enough write the old value to
	      ServoList[i].attach(ServoPins[i]);            // the servo so that it will hold in place if it's applying pressure.
		  delay(10);
		  ServoList[i].write(oldLocation);
		  delay(50);         
		  ServoList[i].detach();   
	      servoLocations[i]=oldLocation;
	  }
    }
    delay(20); // Added to fix left sensor misbehavior reported by Rob.
  }

  latestReading++; // Increment the reading counter so we know where we're at.
  if (latestReading >= ReadingsPerSensor){  // Make sure we don't record more readings than we have space to hold.
    latestReading = ReadingsPerSensor-1;
    // Pop the oldest reading off the list.
    for (i = 0; i < MaxSensors; i++){
      for (j=0; j < ReadingsPerSensor-1; j++){
        sensorReadings[i][j] = sensorReadings[i][j+1];
      }
    }
  }
}

// function: calculateNewDistace(sensorNumber: Which sensor's data to process): Calculated distance in 0-100 range.
// Apply some averaging and smoothing to the recorded distance readings
// to take care of noisy data.
int calculateNewDistace(int sensorNumber){
  int output = SensorFar;                      // Default value is the furthest distance.

  float weightingFactor = 0.5;                 // How fast the reading's importance tapers off in time. (1= no taper, 0 = divide by zero error.)
  float flickerFactor = 30;                    // When the change is greater than this, ignore it unless its two in a row. (It's probably noise.)

  if (latestReading >= ReadingsPerSensor-1) {  // Only do this if we have a full set of readings to sample.
    int total = 0;                             // Average them with a weighting.
    float currentWeight = 1;                   // New readings count more than older readings.
    float percentagePossible = 0;
    boolean flickered = false;
    for (int i=ReadingsPerSensor-1; i >=0 ;i--){   // Check for flicker (This reduces jitter with something right on the threshold.)
      flickered = false;
      if (i==ReadingsPerSensor-1){
        if ((abs(sensorReadings[sensorNumber][i])-abs(sensorReadings[sensorNumber][i-1]) > flickerFactor) &&
           (abs(sensorReadings[sensorNumber][i-1])-abs(sensorReadings[sensorNumber][i-2]) > flickerFactor)){
          flickered = true;
        }
      }
      if (flickered==false){
        total += (sensorReadings[sensorNumber][i] * currentWeight);
        percentagePossible += currentWeight;
        currentWeight *= weightingFactor;
      }
    }
    output = total / percentagePossible;
  }
  return output;
}
// function: getDistance
// Take a sensor number (not pin number) and returns an int in the 0-100 range
// 0 = closest, 100= furthest.  (It's a percentage of the distance that the software
//
// Note: Function is designed to be generic so that it can be swapped out for
//       different kinds of ranging sensors.
//       This version of the function is made for Parallax PING))) sensors
//       For more info see http://arduino.cc/en/Tutorial/Ping
//                     and http://www.parallax.com/tabid/768/ProductID/92/Default.aspx
int getDistance(int sensorNumber){
  long duration;   // How long it takes a sonic pulse to reflect back.
  int out;         // The value we send back from the function

  // Initialize the sensor and tell it to send out a ping.
  pinMode(RangingPins[sensorNumber], OUTPUT);
  digitalWrite(RangingPins[sensorNumber], LOW);
  delayMicroseconds(2);
  digitalWrite(RangingPins[sensorNumber], HIGH);
  delayMicroseconds(5);
  digitalWrite(RangingPins[sensorNumber], LOW);

  // Read the time in milliseconds until the value comes back.
  pinMode(RangingPins[sensorNumber], INPUT);
  duration = pulseIn(RangingPins[sensorNumber], HIGH);

  // Trim the data into minimums and maximums and map it to the 0-100 output range.
  duration = constrain(duration, SenorClose, SensorFar);
  out = map(duration,  SenorClose, SensorFar, 0, 100);
  return out;
}