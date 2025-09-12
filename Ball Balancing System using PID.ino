#include <Arduino_BuiltIn.h>

#define SRVO_PIN 9
#include <Servo.h>
#include <SimpleKalmanFilter.h>

Servo myservo;
int angle = 90;

#define ECHO1_PIN 2
#define TRIG1_PIN 3
#define ECHO2_PIN 5
#define TRIG2_PIN 6

#define MIN_ECHO 0
#define MAX_ECHO 1595

#define DELAY_PROXY 10
#define DELAY_SERVO 10

#define ECHO_2_MM 0.172

const long echoLength = MAX_ECHO - MIN_ECHO;

long duration1 = 0, duration2 = 0;
long distance1 = 0, distance2 = 0;

#define KI_PIN A1
#define KD_PIN A2

#define K_P 2.5
#define K_I 1.5
#define K_D 1.5

#define K_UPPER_LIMIT 2
#define K_LOWER_LIMIT 0

float proportional = 0, integral = 0, derivative = 0;
float dt, last_time = 0, now;
float previous = 0, output = 0;

float delta = 0;
float kp = K_P, ki = K_I, kd = K_D;

float time;

SimpleKalmanFilter kalman1(2, 2, 0.01);
SimpleKalmanFilter kalman2(2, 2, 0.01);

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(DELAY_SERVO);
    myservo.attach(SRVO_PIN);
    myservo.write(90);
    delay(DELAY_SERVO);
    delay(1000);
    pinMode(ECHO1_PIN, INPUT);
    pinMode(TRIG1_PIN, OUTPUT);
    pinMode(ECHO2_PIN, INPUT);
    pinMode(TRIG2_PIN, OUTPUT);
    pinMode(KI_PIN, INPUT);
    pinMode(KD_PIN, INPUT);
    delay(2000);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    delay(100);
    time = micros() / 1e6;
    proxyUltraSonic();
    // if (distance1 == ((echoLength / 2) * ECHO_2_MM)) 
    //   delta = PID(-distance2);
    // else if (distance2 == ((echoLength / 2) * ECHO_2_MM))
    //   delta = PID(distance1);
    // else
    //   delta = PID(distance1 - distance2);
    delta = PID(distance1 - distance2);
    angle = map(delta, (-echoLength), (echoLength), 10, 170);
    myservo.write(int(170 - angle));
    delay(DELAY_SERVO);
    printData();
}

void proxyUltraSonic() {
    digitalWrite(TRIG1_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG1_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG1_PIN, LOW);
    duration1 = pulseIn(ECHO1_PIN, HIGH);
    delay(DELAY_PROXY);
    
    digitalWrite(TRIG2_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG2_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG2_PIN, LOW);
    duration2 = pulseIn(ECHO2_PIN, HIGH);
    delay(DELAY_PROXY);
    
    if (duration1 > 1500) duration1 = echoLength / 2;
    if (duration2 > 1500) duration2 = echoLength / 2;
    
    duration1 = kalman1.updateEstimate(duration1);
    duration2 = kalman2.updateEstimate(duration2);
    
    distance1 = duration1 * ECHO_2_MM;
    distance2 = duration2 * ECHO_2_MM;
}

float PID(float error) {
    ki = float(map(analogRead(KI_PIN), 0, 1023, K_LOWER_LIMIT * 100, K_UPPER_LIMIT * 100)) / 100;
    kd = float(map(analogRead(KD_PIN), 0, 1023, K_LOWER_LIMIT * 100, K_UPPER_LIMIT * 100)) / 100;
    now = millis();
    dt = (now - last_time) / 1000.0;
    last_time = now;
    
    proportional = error;
    integral += error * dt;
    derivative = (error - previous) / dt;
    previous = error;
    
    float output = (kp * proportional) + (ki * integral) + (kd * derivative);
    
    if (ki > 0) {
        integral = constrain(integral, -echoLength / ki, echoLength / ki);
    } else {
        integral = 0;
    }
    output = constrain(output, -echoLength, echoLength);
    return output;
}

void printData() {
    Serial.print("Time:");
    Serial.print(time);
    Serial.print(", Delta_mm:");
    Serial.print(distance1 - distance2);
    Serial.print(", PID_Output_deg:");
    Serial.print(angle);
    Serial.print(", distance1:");
    Serial.print(distance1);
    Serial.print(", distance2:");
    Serial.println(distance2);
}