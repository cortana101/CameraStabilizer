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

#define PCONST -0.0001
#define ICONST -0.000001
#define DCONST -0.005
#define BRAKEZONE 15 // signal corresponding to when we detect braking
#define BRAKEREDUCTIONFACTOR 0.2 // scale factor for braking power
#define SPINDOWNTHRESHOLD 99999

#define CALIBRATEMOTOR false // flag to indicate if we are in the mode of calibrating the motor
 
Servo motor;  // Use a servo to control the PWM of the speed controller
Servo servo;
 
int pos = 0;    
int servoPos = 0;
int currentlyRunning = 0;
int potValue = 0;
int potPin = A0;
int changeIntervalCount = 0;
float atRestZ = 0;
double desiredPosition = POSNEUTRAL;

// Values used in the PID Calculation
double currentDelta = 0; // The current position (P)
double previousDelta = 0; // The previous position, used to calculate the instantaneous speed (D)
double totalDelta = 0; // The total delta since start of operation (I)
 
void setup() 
{ 
  Serial.begin(57600);
  motor.attach(9);  // attaches the servo on pin 9 to the servo object 
  servo.attach(11);
  Wire.begin();
  
  // Set the device to read gyro output on XYZ
  SendOneByte(GYROADDRESS, byte(0x12), byte(0x70));

  // Sets the base sampling rate to 1kHz
  SendOneByte(GYROADDRESS, byte(0x16), byte(0x01));  

  // Sets the sample rate divider to 10 (equates to roughly 100 samples per second)
  SendOneByte(GYROADDRESS, byte(0x15), byte(0x0A));
  
  if (CALIBRATEMOTOR)
  {
    CalibrateMotor();
  }
   
  motor.write(POSNEUTRAL);
  
  delay(1 * 1000);
 
  atRestZ = SetAtRestZ();
  
  delay(1 * 1000);
  
  desiredPosition = POSNEUTRAL;
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
  
  desiredPosition = GetNextPosition(desiredPosition, currentDelta);
  
  Serial.print(desiredPosition);
  Serial.print(",");
  Serial.print(atRestZ);
  Serial.print(",");
  Serial.print(zout);
  Serial.print(",");
  Serial.println(currentDelta);
  
  int potpos = ((float)potValue / 1023) * 180;    
     
  //motor.write(potpos);   
      
  if (potpos > 170)
  {
    motor.write(POSNEUTRAL);
  }  
  else
  {
    motor.write((int)desiredPosition);
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
double GetNextPosition(double current, double currentDelta)
{

      float speed = currentDelta - previousDelta;
      previousDelta = currentDelta;
      
      totalDelta += currentDelta;
      
      // Now we have all of the variables we need for 
      double positionChange = PCONST * currentDelta + DCONST * speed + ICONST * totalDelta;

      //current += positionChange;
      positionChange *= 10;
      
      if (positionChange < BRAKEZONE && positionChange > -BRAKEZONE && (positionChange + POSNEUTRAL) < current && (positionChange + POSNEUTRAL) > -current)
      {
        positionChange *= BRAKEREDUCTIONFACTOR;
      }
      
      current = positionChange + POSNEUTRAL;
      
      // When positionChange gets close to 0, ie we are close to coming to a stop, we should add a reduction factor because the motor will suddenly have a lot more power
      // due to trying to brake
      
      if (current <= POSMIN)
      {
          current = POSMIN;
      }
      
      if (current >= POSMAX)
      {
          current = POSMAX;
      }
      
      return current;
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

void CalibrateMotor()
{
  motor.write(POSMAX);
  delay(4 * 1000);
  motor.write(POSMIN);
  delay(4 * 1000);
  motor.write(POSNEUTRAL);
  delay(6 * 1000);
}
