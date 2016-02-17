char buffer[16];
char comm_state;
char bufferPos;
float yaw, pitch;

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
  Serial.begin(57600);
//  Serial2.begin(57600);
  yaw = pitch = t_x = t_y = 0;
  t_z = 50;
  comm_state = 0;
  clear_buffer();
}

void loop() {
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    if (comm_state == 0) comm_state = inByte;
    else if (inByte == '\n') {
      switch(comm_state) {
        case 'r':
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
}
