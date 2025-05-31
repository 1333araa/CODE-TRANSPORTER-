#include <ESP32Servo.h>
#include <Ps3Controller.h>

// Motor pins
int ENB = 14;
int IN3 = 33;
int IN4 = 32;
int ENA = 27;
int IN1 = 26;
int IN2 = 25;

const int servoPin1 = 12;  //gripper
const int servoPin2 = 13;  //lift
Servo servo1;
Servo servo2;

bool grip = false; //gripperOn
bool lift = false; //liftUp

int gripperPosition = 80;  //posisi awal gripper
int liftPosition = 90;     //posisi awal lift

// Motor settings
#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000;
const int PWMResolution = 8;
const int PWMCHB = 4;
const int PWMCHA = 5;

// Additional variables for enhanced control
int currentSpeed = MAX_MOTOR_SPEED;  // Variable speed control
bool precisionMode = false;          // Precision mode flag
int battery = 0;                     // Battery level tracking
unsigned long lastMoveTime = 0;      // For movement timeout
const int MOVE_TIMEOUT = 400;        // Movement timeout in ms
const int SERVO_DELAY = 15;
const int slowSpeed = 150;

void handleMovement() {
  float x = (float)Ps3.data.analog.stick.rx / 128.0; 
  float y = -(float)Ps3.data.analog.stick.ly / 128.0;

  // Check if the stick is in a neutral position (close to 0)
  if (abs(x) < 0.1 && abs(y) < 0.1) {
    // Stick released, stop the motors
    rotateMotor(0, 0);
    return;
  }

  // Convert to tank drive
  float leftMotor = y + x;
  float rightMotor = y - x;

  // Normalize speeds
  float maxSpeed = max(abs(leftMotor), abs(rightMotor));
  if (maxSpeed > 1.0) {
    leftMotor /= maxSpeed;
    rightMotor /= maxSpeed;
  }

  // Apply current speed setting
  rotateMotor(rightMotor * currentSpeed, leftMotor * currentSpeed);
}

void notify() {
  handleMovement();


  // motor controls
  if (Ps3.data.button.left) {
    Serial.println("Tombol LEFT ditekan");
    moveMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, 1);
  } else if (Ps3.data.button.right) {
    Serial.println("Tombol RIGHT ditekan");
    moveMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, 1);
  } else if (Ps3.data.button.up) {
    Serial.println("Tombol UP ditekan");
    moveMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, 1);
  } else if (Ps3.data.button.down) {
    Serial.println("Tombol DOWN ditekan");
    moveMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, 1);
  } else {
    // Tidak ada tombol arah ditekan, hentikan motor
    moveMotor(0, 0, 0);
  }

  // gripper and lift controls
  if (Ps3.event.button_down.cross) {
    motorServo(servo2, liftPosition, 10, 2);
  }

  if (Ps3.event.button_down.triangle) {
    motorServo(servo2, liftPosition, -10, 2);
  }

  if (Ps3.event.button_down.square) {
    motorServo(servo1, gripperPosition, -10, 1);
  }

  if (Ps3.event.button_down.circle) {
    motorServo(servo1, gripperPosition, 10, 1);
  }

  // additional controls
  if (Ps3.event.button_down.l1) {
    Serial.println("Started pressing the left trigger button");
    toggleGripper();
  }
  if (Ps3.event.button_down.r1) {
    Serial.println("Started pressing the right trigger button");
    toggleLift();
  }

  if (Ps3.event.button_down.l2) {
    precisionMode = !precisionMode;
    Serial.printf("Precision mode: %s\n", precisionMode ? "ON" : "OFF");
  }

  if (Ps3.event.button_down.r2) {
    if (currentSpeed == slowSpeed) {
      // If the current speed is slow, set to maximum
      currentSpeed = MAX_MOTOR_SPEED;
      Serial.println("Speed set to maximum");
    } else {
      // If the current speed is maximum, set to slow
      currentSpeed = slowSpeed;
      Serial.println("Speed set to slow");
    }
  }

  // Battery monitoring
  if (battery != Ps3.data.status.battery) {
    battery = Ps3.data.status.battery;
    Serial.print("Battery: ");
    if (battery == ps3_status_battery_charging)
      Serial.println("Charging");
    else if (battery == ps3_status_battery_full)
      Serial.println("Full");
    else if (battery == ps3_status_battery_high)
      Serial.println("High");
    else if (battery == ps3_status_battery_low)
      Serial.println("Low");
    else if (battery == ps3_status_battery_dying)
      Serial.println("Critical");
    else if (battery == ps3_status_battery_shutdown)
      Serial.println("Shutting down");
  }
}


void stopMotors() {   //untuk kondisi diam
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

// fungsi buka dan tutup gripper
void toggleGripper() {
  if (!grip) {
    moveServoSlowly(servo1, gripperPosition, 80, SERVO_DELAY);
    grip = true;
    gripperPosition = 80;
  } else {
    moveServoSlowly(servo1, gripperPosition, 20, SERVO_DELAY);
    grip = false;
    gripperPosition = 20;
  }
}

// fungsi naik dan turun lift
void toggleLift() {
  if (!lift) {
    moveServoSlowly(servo2, liftPosition, 0, SERVO_DELAY);
    lift = true;
    liftPosition = 0;
  } else {
    moveServoSlowly(servo2, liftPosition, 90, SERVO_DELAY);
    lift = false;
    liftPosition = 90;
  }
}

// menggerakkan servo gripper dan lift dengan sudut maks dan minsnya
void moveServoSlowly(Servo& servo, int startPos, int endPos, int stepDelay) {
  if (startPos < endPos) {
    for (int pos = startPos; pos <= endPos; pos += 5) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = startPos; pos >= endPos; pos -= 5) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
}

//  menggerakkan servo dengan sudut 10 derajat setiap klik button
void motorServo(Servo& servo, int pos, int change, int name) {
  if (name == 1) {
    pos = constrain(pos + change, 20, 80);
    servo.write(pos);
    gripperPosition = pos;
    if (pos = 80) {
      grip = false;
    }
    if (pos = 20) {
      grip = true;
    }
  }
  if (name == 2) {
    pos = constrain(pos + change, 0, 90);
    servo.write(pos);
    liftPosition = pos;
    if (pos = 90) {
      lift = false;
    }
    if (pos = 0) {
      lift = true;
    }
  }
}

// menggerakkan motor dari button
void moveMotor(int motorL, int motorR, bool move) {
  if (move == 1) {
    rotateMotor(motorL, motorR);
  } else if (move == 0) {
    rotateMotor(0, 0);
  }
}

// menggerakkan motor dari toogle
void rotateMotor(int rightMotor, int leftMotor) {
  // Apply precision mode if enabled
  if (precisionMode) {
    rightMotor = rightMotor * 0.5;
    leftMotor = leftMotor * 0.5;
  }

  // Right motor direction
  if (rightMotor < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (rightMotor > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Left motor direction
  if (leftMotor < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (leftMotor > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Set motor speeds
  ledcWrite(PWMCHB, abs(rightMotor));
  ledcWrite(PWMCHA, abs(leftMotor));
}

void setUpPinModes() {
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  ledcSetup(PWMCHB, PWMFreq, PWMResolution);
  ledcSetup(PWMCHA, PWMFreq, PWMResolution);
  ledcAttachPin(ENB, PWMCHB);
  ledcAttachPin(ENA, PWMCHA);

  rotateMotor(0, 0);
}



void onConnect() {
  Serial.println("PS3 Controller Connected.");
  currentSpeed = MAX_MOTOR_SPEED;
  precisionMode = false;
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  servo1.write(gripperPosition);
  servo2.write(liftPosition);

  // PS3 Controller setup
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("cc:7b:5c:28:a9:b6");  // Replace with your ESP32's MAC address

  Serial.println("Setup completed. Waiting for PS3 controller connection...");
}

void loop() {
  if (!Ps3.isConnected()) {
    rotateMotor(0, 0);  // Stop motors if controller disconnected
    return;
  }

  // Movement timeout - stop motors if no input received
  if (millis() - lastMoveTime > MOVE_TIMEOUT) {
    rotateMotor(0, 0);
  }

  delay(20);  // Small delay to prevent overwhelming the system
}
