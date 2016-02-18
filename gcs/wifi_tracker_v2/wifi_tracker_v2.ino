#include <PID_v1.h>

volatile char buffer[8];
char comm_state, bufferPos;
volatile float yaw, pitch;

double Setpoint, Input, Output, out1, out2;
PID myPID(&Input, &Output, &Setpoint, 1, 0.1, 1, DIRECT);

void clear_buffer() {
  buffer[ 0] = buffer[ 1] = buffer[ 2] = buffer[ 3] = buffer[ 4] = buffer[ 5] = buffer[ 6] = buffer[ 7] = '\0';
  bufferPos = 0;
}

void setup() {
  clear_buffer();
  comm_state = 0;
  bufferPos = 0;
  yaw = pitch = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-80, 80);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(57600);
}

void loop() {
  Input = yaw;
  Setpoint = 0;
  myPID.Compute();
  out1 = 0;
  out2 = 0;  
  if(Output<0)
     out1 = -Output;
  else if(Output>0)
     out2 = Output;    
  analogWrite(11, out1);
  analogWrite(12, out2);
  delay(100); 
}

void serialEvent() {
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    if (comm_state == 0) comm_state = inByte;
    else if (inByte == '\n') {
      char* cbuffer = (char*)(unsigned long)&buffer[0];
      switch(comm_state) {
        case 'y':
          yaw = atof(cbuffer);
          break;
        case 'p':
          pitch = atof(cbuffer);
          break;
      }
      comm_state = 0;
      clear_buffer();
    }
    else if (bufferPos > 8) {
      clear_buffer();
      comm_state = '_';
    }
    else {
      buffer[bufferPos] = inByte;
      bufferPos++;  
    }
  }
}
