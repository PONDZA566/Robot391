#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>

// Motor Control Pins
int motorA_in1 = 25;
int motorA_in2 = 26;
int motorA_enable = 5;   // EN_A
int motorB_in1 = 27;
int motorB_in2 = 14;
int motorB_enable = 18;  // EN_B

// Servo Control
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50             // Analog servos run at ~50 Hz
#define SERVOMIN_180  150         // Minimum pulse length for 180-degree servo
#define SERVOMAX_180  600         // Maximum pulse length for 180-degree servo
#define STOP1_360     330         // Pulse length to stop first 360-degree servo
#define STOP2_360     330         // Pulse length to stop second 360-degree servo
#define MAX_FORWARD_360 600       // Max pulse for forward rotation
#define MAX_BACKWARD_360 150      // Min pulse for backward rotation
#define STEP_SIZE 15              // Pulse width step size for 360 servos

int servo360_1 = 0;               // Channel for the first 360-degree servo
int servo360_2 = 1;               // Channel for the second 360-degree servo
int servo180_1 = 2;               // Channel for the first 180-degree servo
int servo180_2 = 3;               // Channel for the second 180-degree servo
int angle180_1 = 90;              // Start at middle angle for the first 180-degree servo
int angle180_2 = 0;              // Start at middle angle for the second 180-degree servo
int speed360_1 = STOP1_360;       // Initial pulse for 360-degree servo 1
int speed360_2 = STOP2_360;       // Initial pulse for 360-degree servo 2
int selectedServoChannel = 0;     // Current selected servo channel

// Variables for gamepad state
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
GamepadPtr myGamepad;
bool motorControlMode = true; // true for motor, false for servo

// Variables to store the previous state of buttons
bool prevDpadUp = false;
bool prevDpadDown = false;
bool prevR1 = false;
bool prevR2 = false;
bool prevSquare = false;
bool prevL2 = false;

// Global flag to indicate if the desired gamepad is connected
bool isDesiredGamepadConnected = false;

// Desired MAC address of the gamepad to lock to
const uint8_t targetMAC[] = {0xF4, 0x93, 0x9F, 0x78, 0x6F, 0x3F};  // Replace with your controller's MAC address

// Helper function to compare two MAC addresses
bool compareMAC(const uint8_t* mac1, const uint8_t* mac2) {
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return false;
        }
    }
    return true;
}

// Callback when a gamepad is connected
void onConnected(ControllerPtr ctl) {
    const ControllerProperties& properties = ctl->getProperties();
    
    // Check if the MAC address matches the desired address
    if (compareMAC(properties.btaddr, targetMAC)) {
        myGamepad = ctl;
        isDesiredGamepadConnected = true; // Set the flag to true
        Serial.println("Gamepad connected with matching MAC address!");
    } else {
        Serial.print("Gamepad connected with MAC address: ");
        for (int i = 0; i < 6; i++) {
            Serial.printf("%02X", properties.btaddr[i]);
            if (i < 5) Serial.print(":");
        }
        Serial.println();
        Serial.println("This is not the desired gamepad. Ignoring.");
        
        // Optionally, disconnect the non-matching gamepad
        ctl->disconnect();
    }
}

// Callback when a gamepad is disconnected
void onDisconnected(ControllerPtr ctl) {
    if (ctl == myGamepad) { // Check if the disconnected gamepad is the one we were using
        myGamepad = nullptr; // Clear the pointer
        isDesiredGamepadConnected = false; // Reset the flag
        Serial.println("Authorized gamepad disconnected!");
    } else {
        Serial.println("Unauthorized gamepad disconnected!");
    }
}

void setup() {
    Serial.begin(115200);

    // Motor setup
    pinMode(motorA_in1, OUTPUT);
    pinMode(motorA_in2, OUTPUT);
    pinMode(motorA_enable, OUTPUT);
    pinMode(motorB_in1, OUTPUT);
    pinMode(motorB_in2, OUTPUT);
    pinMode(motorB_enable, OUTPUT);

    // Servo setup
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);  // Set frequency to 50Hz
    delay(10);

    // Bluepad32 setup
    BP32.setup(&onConnected, &onDisconnected);
    BP32.forgetBluetoothKeys(); // Clear saved Bluetooth keys
}

// Function to change the LED color of the gamepad
void changeLEDColor(uint8_t red, uint8_t green, uint8_t blue) {
    // if (myGamepad) {
    //     myGamepad->setLED(red, green, blue);
    // }
}

void loop() {
    BP32.update();

    // Only handle gamepad input if the desired gamepad is connected
    if (isDesiredGamepadConnected && myGamepad) {
        handleGamepadInput();
    }
    
    delay(15); // Delay for smooth operation
}

// Handle gamepad inputs
void handleGamepadInput() {
    bool currentL2 = myGamepad->l2();

    // Toggle between motor and servo control using L2 button
    if (currentL2 && !prevL2) {
        motorControlMode = !motorControlMode;  // Switch mode
        Serial.print("Switched to ");
        Serial.println(motorControlMode ? "Motor Control" : "Servo Control");
    }
    prevL2 = currentL2;

    if (motorControlMode) {
        controlMotors();
    } else {
        controlServos();
    }
}

// Motor control
void controlMotors() {
    int leftY = myGamepad->axisY();   // Forward/Backward control
    int rightX = myGamepad->axisRX(); // Turning control

    float speedDivider = 1.25; // Speed scaling
    int speed = map(abs(leftY), 0, 512, 0, 255) / speedDivider;
    int turningFactor = map(rightX, -512, 512, -255, 255) / speedDivider;

    if (leftY > 250) { // Backward
        analogWrite(motorA_enable, constrain(speed - turningFactor, 0, 255));
        analogWrite(motorB_enable, constrain(speed + turningFactor, 0, 255));

        digitalWrite(motorA_in1, LOW);
        digitalWrite(motorA_in2, HIGH);
        digitalWrite(motorB_in1, LOW);
        digitalWrite(motorB_in2, HIGH);
        Serial.println("Backward");
    } else if (leftY < -250) { // Forward
        analogWrite(motorA_enable, constrain(speed + turningFactor, 0, 255));
        analogWrite(motorB_enable, constrain(speed - turningFactor, 0, 255));

        digitalWrite(motorA_in1, HIGH);
        digitalWrite(motorA_in2, LOW);
        digitalWrite(motorB_in1, HIGH);
        digitalWrite(motorB_in2, LOW);
        Serial.println("Forward");
    } else if (rightX < -250) { // Turn Left
        analogWrite(motorA_enable, constrain(abs(turningFactor), 0, 255));
        analogWrite(motorB_enable, constrain(abs(turningFactor), 0, 255));

        digitalWrite(motorA_in1, LOW);
        digitalWrite(motorA_in2, HIGH);
        digitalWrite(motorB_in1, HIGH);
        digitalWrite(motorB_in2, LOW);
        Serial.println("Turn Left");
    } else if (rightX > 250) { // Turn Right
        analogWrite(motorA_enable, constrain(abs(turningFactor), 0, 255));
        analogWrite(motorB_enable, constrain(abs(turningFactor), 0, 255));

        digitalWrite(motorA_in1, HIGH);
        digitalWrite(motorA_in2, LOW);
        digitalWrite(motorB_in1, LOW);
        digitalWrite(motorB_in2, HIGH);
        Serial.println("Turn Right");
    } else { // Stop motors
        analogWrite(motorA_enable, 0);
        analogWrite(motorB_enable, 0);
        Serial.println("Stop");
    }
}

// Servo control
void controlServos() {
    bool currentDpadUp = myGamepad->dpad() & DPAD_UP;
    bool currentDpadDown = myGamepad->dpad() & DPAD_DOWN;
    bool currentR2 = myGamepad->r2();
    bool currentSquare = myGamepad->a();  // Stop button
    bool currentR1 = myGamepad->r1();     // Channel change using R1

    // Rotate all servos clockwise when D-pad up is pressed
    if (currentDpadUp) {
        rotateAllServosClockwise();
    } else if (!currentDpadUp && prevDpadUp) {
        stopAllServos();  // Stop servos when D-pad Up is released
        Serial.println("Stopped servos after D-pad Up released.");
    }
    prevDpadUp = currentDpadUp;

    // Rotate all servos counterclockwise when D-pad down is pressed
    if (currentDpadDown) {
        rotateAllServosCounterclockwise();
    } else if (!currentDpadDown && prevDpadDown) {
        stopAllServos();  // Stop servos when D-pad Down is released
        Serial.println("Stopped servos after D-pad Down released.");
    }
    prevDpadDown = currentDpadDown;

    // Switch between servo channels using R2
    if (currentR2 && !prevR2) {
        selectedServoChannel = (selectedServoChannel + 1) % 4;  // Cycle between servo channels
        Serial.print("Selected servo channel: ");
        Serial.println(selectedServoChannel);

        // Change color based on selected servo channel
        switch (selectedServoChannel) {
            case 0:
                myGamepad->setColorLED(0, 0, 255); // blue
                break;
            case 1:
                myGamepad->setColorLED(0, 255, 0); // green
                break;
            case 2:
                myGamepad->setColorLED(255, 255, 0); // yellow
                break;
            case 3:
                myGamepad->setColorLED(255, 0, 0); // red
                break;
        }
    }
    prevR2 = currentR2;

    // Switch between servo channels using R1 (additional control)
    if (currentR1 && !prevR1) {
        selectedServoChannel = (selectedServoChannel - 1 + 4) % 4;  // Cycle between servo channels
        Serial.print("Switched to servo channel: ");
        Serial.println(selectedServoChannel);

         // Change color based on selected servo channel
        switch (selectedServoChannel) {
            case 0:
                myGamepad->setColorLED(0, 0, 255); // blue
                break;
            case 1:
                myGamepad->setColorLED(0, 255, 0); // green
                break;
            case 2:
                myGamepad->setColorLED(255, 255, 0); // yellow
                break;
            case 3:
                myGamepad->setColorLED(255, 0, 0); // red
                break;
        }
    }
    prevR1 = currentR1;

    // Stop all servos when Square is pressed
    if (currentSquare && !prevSquare) {
        stopAllServos();
        Serial.println("All servos stopped.");
    }
    prevSquare = currentSquare;
}

// Functions to rotate servos
void rotateAllServosClockwise() {
    switch (selectedServoChannel) {
        case 0:
            speed360_1 = 390;  // Max forward for 360-degree servo 1
            pwm.setPWM(servo360_1, 0, speed360_1);  // Update PWM for 360-degree servo 1
            Serial.println("Rotating SERVO360_1 clockwise.");
            break;
        case 1:
            speed360_2 = 550;  // Max forward for 360-degree servo 2
            pwm.setPWM(servo360_2, 0, speed360_2);  // Update PWM for 360-degree servo 2
            Serial.println("Rotating SERVO360_2 clockwise.");
            break;
        case 2:
            angle180_1 = min(180, angle180_1 + 1);  // Increase angle for 180-degree servo 1
            pwm.setPWM(servo180_1, 0, map(angle180_1, 0, 180, SERVOMIN_180, SERVOMAX_180));  // Update PWM for 180-degree servo 1
            Serial.println("Rotating 180-degree servo 1 clockwise.");
            break;
        case 3:
            angle180_2 = min(37, angle180_2 + 1);  // Increase angle for 180-degree servo 2
            pwm.setPWM(servo180_2, 0, map(angle180_2, 0, 180, SERVOMIN_180, SERVOMAX_180));  // Update PWM for 180-degree servo 2
            Serial.println("Rotating 180-degree servo 2 clockwise.");
            Serial.println(angle180_2);
            break;
    }
}

void rotateAllServosCounterclockwise() {
    switch (selectedServoChannel) {
        case 0:
            speed360_1 = 255;  // Max backward for 360-degree servo 1
            pwm.setPWM(servo360_1, 0, speed360_1);  // Update PWM for 360-degree servo 1
            Serial.println("Rotating SERVO360_1 counterclockwise.");
            break;
        case 1:
            speed360_2 = 200;  // Max backward for 360-degree servo 2
            pwm.setPWM(servo360_2, 0, speed360_2);  // Update PWM for 360-degree servo 2
            Serial.println("Rotating SERVO360_2 counterclockwise.");
            break;
        case 2:
            angle180_1 = max(0, angle180_1 - 1);  // Decrease angle for 180-degree servo 1
            pwm.setPWM(servo180_1, 0, map(angle180_1, 0, 180, SERVOMIN_180, SERVOMAX_180));  // Update PWM for 180-degree servo 1
            Serial.println("Rotating 180-degree servo 1 counterclockwise.");
            break;
        case 3:
            angle180_2 = max(0, angle180_2 - 1);  // Decrease angle for 180-degree servo 2
            pwm.setPWM(servo180_2, 0, map(angle180_2, 0, 180, SERVOMIN_180, SERVOMAX_180));  // Update PWM for 180-degree servo 2
            Serial.println("Rotating 180-degree servo 2 counterclockwise.");
            Serial.println(angle180_2);
            break;
    }
}

// Stop all servos
void stopAllServos() {
    pwm.setPWM(servo360_1, 0, STOP1_360);
    pwm.setPWM(servo360_2, 0, STOP2_360);
    Serial.println("Stopped all servos.");
}
