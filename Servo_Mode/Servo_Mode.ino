#define BLYNK_PRINT Serial
//#define SERVO_MOTION
#define PIN_SERVO1 25
#define PIN_SERVO2 25
#define PIN_RED 23
#define PIN_GREEN 18
#define TX1_pin  10
#define RX1_pin  9


#define SERVO_PWM_MIN     1000
#define SERVO_PWM_MAX     2000
#define SERVO_ANGLE_MIN   0
#define SERVO_ANGLE_MAX   180
#define SERVO_PWM_STOP    100

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

/******************************************************************************/
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "TP-Link_72E2";
char pass[] = "XXXXXX";

char auth[] = "your blynk auth code";

/******************************************************************************/
#define VIRTUAL_PIN_CURRENT    1
#define VIRTUAL_PIN_VELOCITY   2
#define VIRTUAL_PIN_DIRECTION  3
#define VIRTUAL_PIN_ENABLE     4
#define VIRTUAL_PIN_DURATION   5
#define VIRTUAL_PIN_STARTANGLE 6
#define VIRTUAL_PIN_ENDANGLE   7

BlynkTimer timer;
Servo rotServo;
#ifdef SERVO_MOTION
Servo motionServo;
#endif

unsigned long prevMillis;
unsigned long startMillis;
unsigned long endMillis;
bool direction = true;
bool enable = false;
int duration = 15;
int startAngle = 90;
int endAngle = 180;
int currAngle;
int greenLedStatus = HIGH;
int targetVelocity;

/******************************************************************************/
// These functions are called whenever the corresponding virtual pin is updated
// in the Blynk app

BLYNK_WRITE(VIRTUAL_PIN_CURRENT)
{
  Serial.print("New MaxCurrent set: ");
  Serial.println(param.asInt());
}

BLYNK_WRITE(VIRTUAL_PIN_VELOCITY)
{
  Serial.print("New TargetVelocity set: ");
  Serial.println(param.asInt());

  targetVelocity = param.asInt();
  if (enable)
  {
#ifdef SERVO_MOTION
    motionServo.write(targetVelocity);
#endif
  }
}

BLYNK_WRITE(VIRTUAL_PIN_DIRECTION)
{
  Serial.println("Changed Direction");
  
  direction = param.asInt() != 0;
}

BLYNK_WRITE(VIRTUAL_PIN_ENABLE)
{
  enable = param.asInt() != 0;
  
  if (enable)
  {
    Serial.println("Enable Motor: True");

    currAngle = startAngle;
    startMillis = millis();
    endMillis = startMillis + (duration * 1000);

#ifdef SERVO_MOTION
    motionServo.write(targetVelocity);
#endif    
  }
  else
  {
    Serial.println("Enable Motor: False");
#ifdef SERVO_MOTION
    motionServo.write(SERVO_PWM_STOP);
#endif    
  }
}

BLYNK_WRITE(VIRTUAL_PIN_DURATION)
{
  duration = param.asInt();
  Serial.print("Duration is ");
  Serial.print(duration);
  Serial.println(" s");
}

BLYNK_WRITE(VIRTUAL_PIN_STARTANGLE)
{
  startAngle = param.asInt();
  Serial.print("Start angle is ");
  Serial.print(startAngle);
  Serial.println();

  rotServo.write(startAngle);
}

BLYNK_WRITE(VIRTUAL_PIN_ENDANGLE)
{
  endAngle = param.asInt();
  Serial.print("End angle is ");
  Serial.print(endAngle);
  Serial.println();
}

// Called once per second by the Blynk timer
void periodicJob()
{
  if (enable)
  {
    if (millis() >= endMillis)
    {
      enable = false;
#ifdef SERVO_MOTION
      motionServo.write(SERVO_PWM_STOP);
#endif      
    }
  }

  if (enable)
  {
    Serial.print("Periodic job: currAngle: ");
    Serial.print(currAngle);
    Serial.println();
    
    greenLedStatus = (greenLedStatus == HIGH)? LOW : HIGH;
    digitalWrite(PIN_GREEN, greenLedStatus);
    
    unsigned long delta = millis() - startMillis;
    currAngle = startAngle + (((endAngle - startAngle) * delta) / (duration * 1000));
    rotServo.write(currAngle);   
  }
}

void setup()
{
  // Debug console
  Serial.begin(115200);

  // TMC2300 IC UART connection
  //Serial1.begin(115200, SERIAL_8N1, RX1_pin, TX1_pin);

  // Status LED
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_RED, OUTPUT);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  rotServo.setPeriodHertz(50);    // standard 50 hz servo
  rotServo.attach(PIN_SERVO1, SERVO_PWM_MIN, SERVO_PWM_MAX); // attaches the servo on pin 18 to the servo object

  rotServo.write(0);
  delay(1000);
  rotServo.write(180);
  delay(1000);
  rotServo.write(90);

#ifdef SERVO_MOTION
  motionServo.setPeriodHertz(50);    // standard 50 hz servo
  motionServo.attach(PIN_SERVO2, SERVO_PWM_MIN, SERVO_PWM_MAX); // attaches the servo on pin 18 to the servo object
#endif

  rotServo.write(0);
  delay(1000);
  rotServo.write(180);
  delay(1000);
  rotServo.write(90);

#ifdef SERVO_MOTION
  motionServo.write(SERVO_PWM_STOP);
#endif

  // Connect to the WiFi access point
  Serial.println("Connecting to the WiFi access point");
#ifdef BLYNK_BLE
  Blynk.setDeviceName("Blynk");

  Blynk.begin(auth);
#else
  // Connect to the WiFi access point
  Serial.println("Connecting to the WiFi access point");
  Blynk.connectWiFi(ssid, pass);

  // Connect to the Blynk server
  Serial.println("Authentificating with the Blynk server");
  Blynk.config(auth);
#endif

  Serial.println("Initialization complete");

  // Start the timer for the periodic function
  timer.setInterval(100L, periodicJob);

  prevMillis = millis();
}

void loop()
{
  Blynk.run();
  timer.run();

  if (Serial.available())
  {
    int c = Serial.read();

    if (c == '1')
    {
      Serial.println(targetVelocity);
      targetVelocity += 5;
      Serial.println(targetVelocity);
#ifdef SERVO_MOTION
      motionServo.write(targetVelocity);
#endif      
    }
    else if (c == '2')
    {
      Serial.println(targetVelocity);
      targetVelocity -= 5;
      Serial.println(targetVelocity);
#ifdef SERVO_MOTION
      motionServo.write(targetVelocity);
#endif      
    }
  }
}
