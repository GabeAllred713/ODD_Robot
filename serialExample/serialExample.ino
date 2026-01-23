int motor1Command = 0;
int motor2Command = 0;
double motor1Encoder = 0;
double motor2Encoder = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
}

bool readSerialMessage() {
  int motor1Serial = Serial.parseInt();
  int motor2Serial = Serial.parseInt();

  if (!Serial.find('\n')) return false;

  motor1Command = motor1Serial;
  motor2Command = motor2Serial;
  return true;
}

void writeSerialMessage() {
  Serial.print(motor1Encoder);
  Serial.print(",");
  Serial.println(motor2Encoder);
}

void loop() {
  motor1Encoder += motor1Command*0.001;
  motor2Encoder += motor2Command*0.001;
  
  if (Serial.available() > 0) {
    if (readSerialMessage()) {
      writeSerialMessage();
    }
  }
}
