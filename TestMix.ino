#include <Ps3Controller.h>
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;

int player = 0;
int battery = 0;
int pos1 = 0;
int pos2 = 90;


const int ENA = 27;
const int IN1 = 26;
const int IN2 = 25;

const int ENB = 35;
const int IN3 = 33;
const int IN4 = 32;

const int PWMFreq = 1000;     // 1 kHz
const int PWMResolution = 8;  // 8-bit: 0–255
const int PWMChannelA = 0;    // Untuk ENA
const int PWMChannelB = 1;    // Untuk ENB

#define MAX_SPEED 100

void notify()

{
  //--- Digital cross/square/triangle/circle button events ---
  if (Ps3.event.button_down.cross) {
    Serial.println("Started pressing the cross button");
    servo1.write(90);
    pos1 = 90;
    delay(10);
  }

  if (Ps3.event.button_up.cross)
    Serial.println("Released the cross button");

  if (Ps3.event.button_down.square) {
    Serial.println("Started pressing the square button");
    pos2 = constrain(pos2 - 15, 0, 90);
    servo2.write(pos2);
    delay(10);
  }

  if (Ps3.event.button_up.square)
    Serial.println("Released the square button");

  if (Ps3.event.button_down.triangle) {
    Serial.println("Started pressing the triangle button");
    pos1 = constrain(pos1 - 15, 0, 90);
    servo1.write(pos1);
    delay(10);
  }

  if (Ps3.event.button_up.triangle)
    Serial.println("Released the triangle button");

  if (Ps3.event.button_down.circle) {
    Serial.println("Started pressing the circle button");
    pos2 = constrain(pos2 + 15, 0, 90);
    servo2.write(pos2);
    delay(10);
  }

  if (Ps3.event.button_up.circle)
    Serial.println("Released the circle button");

  //--------------- Digital D-pad button events --------------
  if (Ps3.event.button_down.up)
    Serial.println("Started pressing the up button");

  if (Ps3.event.button_up.up)
    Serial.println("Released the up button");

  if (Ps3.event.button_down.right)
    Serial.println("Started pressing the right button");

  if (Ps3.event.button_up.right)
    Serial.println("Released the right button");

  if (Ps3.event.button_down.down)
    Serial.println("Started pressing the down button");

  if (Ps3.event.button_up.down)
    Serial.println("Released the down button");

  if (Ps3.event.button_down.left)
    Serial.println("Started pressing the left button");

  if (Ps3.event.button_up.left)
    Serial.println("Released the left button");

  //------------- Digital shoulder button events -------------
  if (Ps3.event.button_down.l1)
    Serial.println("Started pressing the left shoulder button");

  if (Ps3.event.button_up.l1)
    Serial.println("Released the left shoulder button");

  if (Ps3.event.button_down.r1)
    Serial.println("Started pressing the right shoulder button");

  if (Ps3.event.button_up.r1)
    Serial.println("Released the right shoulder button");

  //-------------- Digital trigger button events -------------
  if (Ps3.event.button_down.l2)
    Serial.println("Started pressing the left trigger button");

  if (Ps3.event.button_up.l2)
    Serial.println("Released the left trigger button");

  if (Ps3.event.button_down.r2)
    Serial.println("Started pressing the right trigger button");

  if (Ps3.event.button_up.r2)
    Serial.println("Released the right trigger button");

  //--------------- Digital stick button events --------------
  if (Ps3.event.button_down.l3)
    Serial.println("Started pressing the left stick button");

  if (Ps3.event.button_up.l3)
    Serial.println("Released the left stick button");

  if (Ps3.event.button_down.r3)
    Serial.println("Started pressing the right stick button");

  if (Ps3.event.button_up.r3)
    Serial.println("Released the right stick button");

  //---------- Digital select/start/ps button events ---------
  if (Ps3.event.button_down.select)
    Serial.println("Started pressing the select button");

  if (Ps3.event.button_up.select)
    Serial.println("Released the select button");

  if (Ps3.event.button_down.start)
    Serial.println("Started pressing the start button");

  if (Ps3.event.button_up.start)
    Serial.println("Released the start button");

  if (Ps3.event.button_down.ps)
    Serial.println("Started pressing the PlayStation button");

  if (Ps3.event.button_up.ps)
    Serial.println("Released the PlayStation button");

  //---------------- Analog stick value events ---------------
  int y = Ps3.data.analog.stick.ly;  // Left stick Y axis
  int x = Ps3.data.analog.stick.rx;  // Right stick X axis

  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" | X: ");
  Serial.println(x);

  // Threshold agar tidak terlalu sensitif
  if (y <= -50) {
    // Maju
    setMotor(MAX_SPEED, MAX_SPEED);
  } else if (y >= 50) {
    // Mundur
    setMotor(-MAX_SPEED, -MAX_SPEED);
  } else if (x >= 50) {
    // Belok kanan
    setMotor(MAX_SPEED, -MAX_SPEED);
  } else if (x <= -50) {
    // Belok kiri
    setMotor(-MAX_SPEED, MAX_SPEED);
  } else {
    // Stop
    setMotor(0, 0);
  }



  //--------------- Analog D-pad button events ----------------

  if (abs(Ps3.event.analog_changed.button.up)) {

    Serial.print("Pressing the up button: ");

    Serial.println(Ps3.data.analog.button.up, DEC);
  }



  if (abs(Ps3.event.analog_changed.button.right)) {

    Serial.print("Pressing the right button: ");

    Serial.println(Ps3.data.analog.button.right, DEC);
  }



  if (abs(Ps3.event.analog_changed.button.down)) {

    Serial.print("Pressing the down button: ");

    Serial.println(Ps3.data.analog.button.down, DEC);
  }



  if (abs(Ps3.event.analog_changed.button.left)) {

    Serial.print("Pressing the left button: ");

    Serial.println(Ps3.data.analog.button.left, DEC);
  }



  //---------- Analog shoulder/trigger button events ----------

  if (abs(Ps3.event.analog_changed.button.l1)) {

    Serial.print("Pressing the left shoulder button: ");

    Serial.println(Ps3.data.analog.button.l1, DEC);
  }



  if (abs(Ps3.event.analog_changed.button.r1)) {

    Serial.print("Pressing the right shoulder button: ");

    Serial.println(Ps3.data.analog.button.r1, DEC);
  }



  if (abs(Ps3.event.analog_changed.button.l2)) {
    Serial.print("Pressing the left trigger button: ");
    Serial.println(Ps3.data.analog.button.l2, DEC);
  }

  if (abs(Ps3.event.analog_changed.button.r2)) {
    Serial.print("Pressing the right trigger button: ");
    Serial.println(Ps3.data.analog.button.r2, DEC);
  }

  //---- Analog cross/square/triangle/circle button events ----
  if (abs(Ps3.event.analog_changed.button.triangle)) {
    Serial.print("Pressing the triangle button: ");
    Serial.println(Ps3.data.analog.button.triangle, DEC);
  }

  if (abs(Ps3.event.analog_changed.button.circle)) {
    Serial.print("Pressing the circle button: ");
    Serial.println(Ps3.data.analog.button.circle, DEC);
  }

  if (abs(Ps3.event.analog_changed.button.cross)) {
    Serial.print("Pressing the cross button: ");
    Serial.println(Ps3.data.analog.button.cross, DEC);
  }

  if (abs(Ps3.event.analog_changed.button.square)) {
    Serial.print("Pressing the square button: ");
    Serial.println(Ps3.data.analog.button.square, DEC);
  }

  //---------------------- Battery events ---------------------
  if (battery != Ps3.data.status.battery) {
    battery = Ps3.data.status.battery;
    Serial.print("The controller battery is ");
    if (battery == ps3_status_battery_charging) Serial.println("charging");
    else if (battery == ps3_status_battery_full) Serial.println("FULL");
    else if (battery == ps3_status_battery_high) Serial.println("HIGH");
    else if (battery == ps3_status_battery_low) Serial.println("LOW");
    else if (battery == ps3_status_battery_dying) Serial.println("DYING");
    else if (battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
    else Serial.println("UNDEFINED");
  }
}

void onConnect() {
  Serial.println("Connected.");
}

void setup()
{
  Serial.begin(115200);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("01:02:03:04:05:06");

  servo1.attach(13);
  servo2.attach(12);
  servo1.write(pos1);
  servo2.write(pos2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWMChannelA, PWMFreq, PWMResolution);
  ledcAttachPin(ENA, PWMChannelA);
  ledcSetup(PWMChannelB, PWMFreq, PWMResolution);
  ledcAttachPin(ENB, PWMChannelB);
  setMotor(0, 0);

  Serial.println("Ready.");
}

void loop()
{
  if (!Ps3.isConnected())
    return;

  //-------------------- Player LEDs -------------------
  Serial.print("Setting LEDs to player ");
  Serial.println(player, DEC);

  Ps3.setPlayer(player);
  player += 1;
  if (player > 10) player = 0;

  //------ Digital cross/square/triangle/circle buttons ------
  if (Ps3.data.button.cross && Ps3.data.button.down)
    Serial.println("Pressing both the down and cross buttons");

  if (Ps3.data.button.square && Ps3.data.button.left)
    Serial.println("Pressing both the square and left buttons");

  if (Ps3.data.button.triangle && Ps3.data.button.up)
    Serial.println("Pressing both the triangle and up buttons");

  if (Ps3.data.button.circle && Ps3.data.button.right)
    Serial.println("Pressing both the circle and right buttons");

  if (Ps3.data.button.l1 && Ps3.data.button.r1)
    Serial.println("Pressing both the left and right bumper buttons");

  if (Ps3.data.button.l2 && Ps3.data.button.r2)
    Serial.println("Pressing both the left and right trigger buttons");

  if (Ps3.data.button.l3 && Ps3.data.button.r3)
    Serial.println("Pressing both the left and right stick buttons");

  if (Ps3.data.button.select && Ps3.data.button.start)
    Serial.println("Pressing both the select and start buttons");

  delay(2000);
}

void setMotor(int speedA, int speedB) {
  // Motor A (Kanan)
  if (speedA > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speedA < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(PWMChannelA, abs(speedA));

  // Motor B (Kiri)
  if (speedB > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speedB < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  ledcWrite(PWMChannelB, abs(speedB));
}