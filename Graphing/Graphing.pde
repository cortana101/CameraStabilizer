 // This program takes ASCII-encoded strings
 // from the serial port at 9600 baud and graphs them. It expects values in the
 // range 0 to 1023, followed by a newline, or newline and carriage return
 
 // Created 20 Apr 2005
 // Updated 18 Jan 2008
 // by Tom Igoe
 // This example code is in the publiheightc domain.
 
 import processing.serial.*;

 int windowX = 1000;
 int windowY = 900;
  
 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 
 void setup () {
 
 // set the window size:
 size(windowX, windowY);

 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[4], 57600);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(0);
 }
 
 void draw () {
   // Sets up the grid for viewing
   // everything happens in the serialEvent()
 
   stroke(0, 200, 0);
   line(0, height / 3, width, height / 3);
   line(0, height - height / 3, width, height - height / 3); 
 }
 
 
 void serialEvent (Serial myPort) {
   // get the ASCII string:
   String inString = myPort.readString();
    
   if (inString != null) {
     // trim off any whitespace:
     inString = trim(inString);
     
     String[] inStrings = split(inString, ",");
     
      // convert to an int and map to the screen height:
     float inByte = float(inStrings[0]); 
     float inByteY = float(inStrings[1]);
     float inByteZ = float(inStrings[2]);
     float currentDelta = float(inStrings[3]);
     
     print("instring:");
     println(inString);
     print(inByteZ);
     print(",");
     println(currentDelta);
     
     inByte = map(inByte, 0, 180, 0, height / 3);
     
     // The inByte
     stroke(127,34,255);
     //line(xPos, height, xPos, height - inByte);
     point(xPos, height - height / 3 - inByte);
     
     //stroke(150, 255, 30);
     ////line(xPos, height, xPos, height - movingAverage);
     //point(xPos, height - movingAverage);
    
     inByteY = map(inByteY, -1000, 1000, 0, height);
     inByteZ = map(inByteZ, -1000, 1000, 0, height / 3);
     
     //stroke(255, 150, 30);
     //point(xPos, height - inByteY);
     
     stroke(30, 150, 255);
     point(xPos, height / 3 - inByteZ);
     
     float currentDeltaPoint = map(currentDelta, -10000, 10000, 0, height / 3);
     stroke(255, 255, 255);
     point(xPos, height - currentDeltaPoint);
     
     // at the edge of the screen, go back to the beginning:
     if (xPos >= width) {
       xPos = 0;
       background(0); 
     } 
     else {
     // increment the horizontal position:
       xPos++;
     }
   }
 }
 
