/*
Control algorithm notes:

1. Due to the fact that we have to send the controller an Int value, we have to ensure that our PCONST 
   is such that at small delta values it would not be so low as to run into  a rounding error.
   We may need to, for small deltas, implement a delay but position control signal. (ie for small deltas,
   still send a positive or negative signal, but delay computing the next value for a few more clock cycles
   than we are now, to smooth it down from rapidly changing in the middle.


*/

#include <Servo.h> 
#include <Wire.h>
//#include <Math.h>

#define MIN 0
#define MAX 180
#define GYROADDRESS 104
#define changeInterval 600 // change motor every 2 seconds
#define CALIBRATIONTIME 2000 // ms spent at the beginning calibrating the gyro
#define IGNORETHRESHOLD 13
#define POSMIN 0
#define POSMAX 180
#define POSNEUTRAL 90
#define SPINDOWNDELAY 90 // ms between spin up and down intervals

#define PCONST -0.1
#define ICONST 0.0
#define DCONST 0.0001
#define POSITIVEBIAS 1.5 // scale factor for positive movement instead of negative movement to compensate for more powerful slowing down
#define SPINDOWNTHRESHOLD 99999
#define UPDATECYCLECOUNT 10
 
Servo motor;  // Use a servo to control the PWM of the speed controller
Servo servo;
 
int pos = 0;    
int servoPos = 0;
int currentlyRunning = 0;
int potValue = 0;
int potPin = A0;
int changeIntervalCount = 0;
float atRestZ = 0;
int updateCycle = 0;

// Values used in the PID Calculation
double currentDelta = 0; // The current position (P)
double previousDelta = 0; // The previous position, used to calculate the instantaneous speed (D)
double totalDelta = 0; // The total delta since start of operation (I)
 
void setup() 
{ 
  Serial.begin(57600);
  //Serial.begin(9600);
  motor.attach(9);  // attaches the servo on pin 9 to the servo object 
  servo.attach(11);
  Wire.begin();
  
  // Set the device to read gyro output on XYZ
  SendOneByte(GYROADDRESS, byte(0x12), byte(0x70));

  // Sets the base sampling rate to 1kHz
  SendOneByte(GYROADDRESS, byte(0x16), byte(0x01));  

  // Sets the sample rate divider to 10 (equates to roughly 100 samples per second)
  SendOneByte(GYROADDRESS, byte(0x15), byte(0x0A));
   
  motor.write(POSNEUTRAL);
  
  delay(1 * 1000);
 
  atRestZ = SetAtRestZ();
  
  delay(1 * 1000);
  
  pos = POSNEUTRAL;
    
  // Improve the algo to pay more attention to the speed and slow down before hitting the target position
  // Use absolute time based delays rather than delay values like we have now
} 
 
 
void loop() 
{ 
  potValue = analogRead(potPin);
 
   // Change the motor speed from 100 to 150 every 2 seconds
   if (changeIntervalCount > changeInterval)
   {
     if (servoPos == 40)
     {
       servoPos = 80;
     }
     else
     {
       servoPos = 40;
     }
        
     changeIntervalCount = 0;
   }
   
   changeIntervalCount++;
  
  int xout = 0;    
  int yout = 0;
  int zout = 0;

  Wire.beginTransmission(GYROADDRESS);
  Wire.write(byte(0x1D)); // Point to the Xout byte
  Wire.endTransmission();
  
  Wire.requestFrom(GYROADDRESS, 6);

  while(6 <= Wire.available())
  {
      xout = Wire.read();
      xout = xout << 8;
      xout |= Wire.read();
         
      yout = Wire.read();
      yout = yout << 8;
      yout |= Wire.read();
      
      zout = Wire.read();
      zout = zout << 8;
      zout |= Wire.read();
  }
  
  if (fabs(zout - atRestZ) > IGNORETHRESHOLD)
  {
    currentDelta = currentDelta + (zout - atRestZ);
  }
  
  pos = GetNextPosition(pos, currentDelta);
  
  Serial.print(pos);
  Serial.print(",");
  Serial.print(atRestZ);
  Serial.print(",");
  Serial.print(zout);
  Serial.print(",");
  Serial.println(currentDelta);
  
  int potpos = ((float)potValue / 1023) * 180;    
  ////Serial.println(potpos);
     
  //motor.write(potpos);   
      
  if (potpos > 170)
  {
    motor.write(POSNEUTRAL);
  }  
  else
  {
    motor.write(pos);
  } 
  
//  servo.write(servoPos);
  servo.write(0);    
  delay(10);
} 

// Spends a few seconds gathering data initially on startup to determine what the calibrated zero of the gyro should be
float SetAtRestZ()
{
    // 10 ms sampling resolution
    int samplingResolution = 10;
    
    long totalSample = 0;
    int sampleCount = 0;
  
    for (int i = 0; i < (CALIBRATIONTIME / samplingResolution); i++)
    {
        Wire.beginTransmission(GYROADDRESS);
        Wire.write(byte(0x1D)); // Point to the Xout byte
        Wire.endTransmission();
        
        Wire.requestFrom(GYROADDRESS, 6);
      
        int zout = 0;
      
        while(6 <= Wire.available())
        {
            // We dont actually need X and Y out for this prototype, so we just throw it away
            int xout = Wire.read();
            xout = xout << 8;
            xout |= Wire.read();
               
            int yout = Wire.read();
            yout = yout << 8;
            yout |= Wire.read();
            
            zout = Wire.read();
            zout = zout << 8;
            zout |= Wire.read();

            totalSample += zout;
            sampleCount++;
        }
       
        delay(samplingResolution);
    }
    
    return (float)totalSample / sampleCount;
}

// Gets the next signal to send to the controller. Also tracks the delta over time to determine speed and integral
int GetNextPosition(int current, double currentDelta)
{
    updateCycle++;
    
    if (updateCycle < UPDATECYCLECOUNT)
    {
      return current;
    }
    else
    {
      updateCycle = 0;
    
    /*
      if (currentDelta > SPINDOWNTHRESHOLD)
      {
          return GetNextPositionSpinUp(current);
      }
      else if (currentDelta < -SPINDOWNTHRESHOLD)
      {
          return GetNextPositionSpinDown(current);
      }*/
  
      float speed = currentDelta - previousDelta;
      previousDelta = currentDelta;
      
      totalDelta += currentDelta;
      
      // Now we have all of the variables we need for PID
      
      double positionChange = PCONST * currentDelta + DCONST * speed;
      
      // Depending on the style of positionChange, we may need to add a single directional bias, e.g we know that slowing down is
      // much more powerful than speeding up, so when positionChange is negative we should compensate for that a bit
      //if (positionChange > 0)
      //{
      //    positionChange *= POSITIVEBIAS;
      //}
      
      current += (int)positionChange;
      
      if (current <= POSMIN)
      {
          current = POSMIN;
      }
      
      if (current >= POSMAX)
      {
          current = POSMAX;
      }
      
      return current;
      // May need additional logic here to recover from situations where the motor is maxed out or stopped and the delta is not at zero
      // we may need a way to spin up/down very slowly to not overcome the static friction and then apply the proper force to regain position.
    }
}

// Gets the next value to send to the controller on the spin down routine (slowly spin down and snap fast at the end)
int GetNextPositionSpinDown(int current)
{
    delay(SPINDOWNDELAY);
    
    if (current > POSMIN)
    {
        return current - 1;
    }
    else
    {
      return POSMAX;
    }
}

// Gets the next value to send to the controller on the spin up routine
int GetNextPositionSpinUp(int current)
{
    delay(SPINDOWNDELAY);
    
    if (current < POSMAX)
    {
        return current + 1;
    }
    else
    {
        return POSMIN;
    }
}

// Sends one byte to the I2C controlled device
void SendOneByte(byte deviceAdd, byte registerAdd, byte value)
{
    Wire.beginTransmission(deviceAdd);
    Wire.write(registerAdd); // Set register pointer to the register we want to write to
    Wire.write(value); // Set value of this register to the provided value
    Wire.endTransmission();
}
