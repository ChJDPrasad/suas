#include <PID_v1.h>

char buffer[16];
char comm_state;
char bufferPos;
float yaw, pitch;
double Setpoint, Input, Output, out1, out2;

PID myPID(&Input, &Output, &Setpoint,2,0.5,1, DIRECT);
int output1 = 11;
int output2 = 12;

void clear_buffer() {
  bufferPos = 0;
  buffer[ 0] = '\0';
  buffer[ 1] = '\0';
  buffer[ 2] = '\0';
  buffer[ 3] = '\0';
  buffer[ 4] = '\0';
  buffer[ 5] = '\0';
  buffer[ 6] = '\0';
  buffer[ 7] = '\0';
  buffer[ 8] = '\0';
  buffer[ 9] = '\0';
  buffer[10] = '\0';
  buffer[11] = '\0';
  buffer[12] = '\0';
  buffer[13] = '\0';
  buffer[14] = '\0';
  buffer[15] = '\0';
}

void setup() {
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(57600);
//  Serial2.begin(57600);
  yaw = pitch=0;
  comm_state = 0;
  myPID.SetMode(AUTOMATIC);
  clear_buffer();
}

void loop() {
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    if (comm_state == 0) comm_state = inByte;
    else if (inByte == '\n') {
      switch(comm_state) {
        case 'y':
          yaw = atof(buffer);
//          Serial2.println(roll);
          break;
        case 'p':
          pitch = atof(buffer);
//          Serial2.println(pitch);
          break;
      }
      comm_state = 0;
      clear_buffer();
    }
    else {
      buffer[bufferPos] = inByte;  
      bufferPos++;
    }
  }
  Setpoint = 0;
  Input = yaw;
  myPID.Compute();
  
  if (Output < -130)
     Output = -130;
  if (Output >130)
     Output = 130;
     
     
//  Serial2.println(Output);
  delay(10); 
  out1 = 0;
  out2 = 0;  
  
     
  if(Output<0)
     out1 = -Output;
  else if(Output>0)
     out2 = Output;    
  analogWrite(11, out1);
  analogWrite(12, out2);
}
