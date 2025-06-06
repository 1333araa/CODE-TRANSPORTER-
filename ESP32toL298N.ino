int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14;

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 170;

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcWrite(pwmChannel, dutyCycle);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Move DC motor forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  delay(1000);

  // Move DC motor backward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);

  delay(1000);

  //move forward with increasing speed
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  for (dutyCycle = 170; dutyCycle < 255; dutyCycle++) {
    ledcWrite(pwmChannel, dutyCycle);
    delay(20);
  }

  dutyCycle = 170;
  ledcWrite(pwmChannel, dutyCycle);
}