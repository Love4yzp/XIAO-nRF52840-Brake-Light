#include "LSM6DS3.h"
#include "Wire.h"
#include "led_matrix.h"
#include <MahonyAHRS.h>

Mahony filter;

const int THREAD_BICYCLE_DELAY_MS = 100;
const float dt                    = 0.1f;
NonBlockingDelay thread_led(DELAYVAL);
NonBlockingDelay thread_bicycle(THREAD_BICYCLE_DELAY_MS);
NonBlockingDelay thread_print(100);

String getLEDTitle(LED_DIRECTIONS led_dir) {
    switch (led_dir) {
        case LED_DIRECTIONS::UP:
            return "前进^^^";
        case LED_DIRECTIONS::DOWN:
            return "刹车XXX";
        case LED_DIRECTIONS::LEFT:
            return "左转<<<";
        case LED_DIRECTIONS::RIGHT:
            return "右转>>>";
        case LED_DIRECTIONS::FULL:
            return "刹车XXX";
        case LED_DIRECTIONS::SOFT_FULL:
            return "直行^^^";
        default:
            return "";
    }
}

/********* Setup ***********/
led_matrix led;
LED_DIRECTIONS led_dir = LED_DIRECTIONS::FULL;
int led_holdTime       = 0;
bool led_trigger       = false;

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // I2C device address 0x6A

void setup() {
    Serial.begin(115200);
    // Initialize LSM6DS3
    myIMU.settings.tempEnabled = 0;
    if (myIMU.begin() != IMU_SUCCESS) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
    led.begin();
    filter.begin(10);

    CalcImuNoiseFloat();
    delay(500);
}

#define PITCH_THREAD   30.0f  // Valid range: [-30, 30]
#define STANDING_PITCH 5.0f   // the pitch When standing

#define BRAKE_THRESHOLD -5.0f  // Brake acceleration threshold

// As the IMU is changed, The Z-axis replaces the original Y-axis direction

float accOffsetX, accOffsetY, accOffsetZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ = 0;
float accX, accY, accZ, gyroX, gyroY, gyroZ;

float x, y, z, aRoll, aPitch, aHeading;            // three axis acceleration data
double roll = 0.00, pitch = 0.00, heading = 0.00;  // Roll & Pitch are the angles which rotate by the axis X and y

void inline task_bicycle() {
    static float prevAccZ = 0;

    accX  = myIMU.readFloatAccelX() - accOffsetX;  // Left is negative
    accY  = myIMU.readFloatAccelY() - accOffsetY;  // Up is negative
    accZ  = myIMU.readFloatAccelZ() - accOffsetZ;  // forward is negative
    gyroX = myIMU.readFloatGyroX() - gyroOffsetX;
    gyroY = myIMU.readFloatGyroY() - gyroOffsetY;
    gyroZ = myIMU.readFloatGyroZ() - gyroOffsetZ;

    filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

    float accelChangeRate = (accZ - prevAccZ) / (THREAD_BICYCLE_DELAY_MS / 1000.0f);
    prevAccZ              = accZ;
    float Pitch           = filter.getPitch();
    if (fabs(Pitch) < PITCH_THREAD) {  // Valid range: [-30, 30]

        do {
            if (accelChangeRate < BRAKE_THRESHOLD) {
                Serial.println("Brake detected, accelChangeRate: " + String(accelChangeRate) + " < BRAKE_THRESHOLD: " + String(BRAKE_THRESHOLD));
                led_dir      = LED_DIRECTIONS::FULL;
                led_holdTime = 3000;
                led_trigger  = true;
                break;
            }
            if (led_trigger) {
                break;
            }
            if (fabs(Pitch) < STANDING_PITCH) {
                Serial.println("Driving straight or braking");
                led_dir = LED_DIRECTIONS::SOFT_FULL;
                break;
            }
            if(Pitch > 0) {
                led_dir = LED_DIRECTIONS::LEFT;
            } else {
                led_dir = LED_DIRECTIONS::RIGHT;
            }

        } while (0);
    }
}

void inline task_led(void) {
    if (led_trigger && led_holdTime > 0) {
        led_holdTime -= DELAYVAL;       // Ensure that the correct time is subtracted from each call
        led.animate_shift(led_dir, 2);  // Keep displaying brake lights
    } else {
        led_trigger = false;            // Reset trigger
        led.animate_shift(led_dir, 2);  // Update the lights to their current state
    }
}

void inline task_print(void) {
    roll    = filter.getRoll();   // X Front and rear of the car
    pitch   = filter.getPitch();  // Y
    heading = filter.getYaw();    // Z
                                  // Serial.println("Pitch(左右倾斜):" + String(pitch) + ",Roll(车头扬起):" + String(roll) + ",Yaw(中心围绕):" + String(heading));
    Serial.println("LED_DIR>>>" + String(getLEDTitle(led_dir)));
    Serial.println("Pitch(左右倾斜):" + String(pitch) + ",accX:" + String(accX) + ",accY:" + String(accY) + ",accZ:" + String(accZ) + ",gyroX:" + String(gyroX) + ",gyroY:" + String(gyroY) + ",gyroZ:" + String(gyroZ));
}

void loop() {

    if (thread_bicycle.check()) {
        task_bicycle();
    }
    if (thread_led.check()) {
        task_led();
    }
    if (thread_print.check()) {
        task_print();
    }
}

void CalcImuNoiseFloat() {
    Serial.print(" **** Function: CalcImuNoiseFloat()");

    float xa = 0, ya = 0, za = 0, xg = 0, yg = 0, zg = 0;
    int iterations = 200;

    // Acc offset
    for (int i = 0; i < iterations; i++) {
        xa += myIMU.readFloatAccelX();
        ya += myIMU.readFloatAccelY();
        za += myIMU.readFloatAccelZ();
    }

    // Gyro offset
    for (int i = 0; i < iterations; i++) {
        xg += myIMU.readFloatGyroX();
        yg += myIMU.readFloatGyroY();
        zg += myIMU.readFloatGyroZ();
    }

    accOffsetX  = xa / iterations;
    accOffsetY  = ya / iterations - 1;  // Assuming 1g in the Y direction at rest
    accOffsetZ  = za / iterations;
    gyroOffsetX = xg / iterations;
    gyroOffsetY = yg / iterations;
    gyroOffsetZ = zg / iterations;

    Serial.print(" accOffsetX: ");
    Serial.print(accOffsetX, 4);
    Serial.print(" accOffsetY: ");
    Serial.print(accOffsetY, 4);
    Serial.print(" accOffsetZ: ");
    Serial.print(accOffsetZ, 4);
    Serial.println();
    Serial.print("*** Gyro offSet ***");
    Serial.print(" gyroOffsetX: ");
    Serial.print(gyroOffsetX, 4);
    Serial.print(" gyroOffsetY: ");
    Serial.print(gyroOffsetY, 4);
    Serial.print(" gyroOffsetZ: ");
    Serial.print(gyroOffsetZ, 4);
    Serial.println();
}

void CalcImuNoiseRaw() {
    Serial.print(" **** Function: CalcImuNoiseRaw()");

    int16_t xa, ya, za, xg, yg, zg;
    int iterations = 200;
    xa             = 0;
    ya             = 0;
    za             = 0;

    // ----------- Acc offset
    for (int i = 0; i < iterations; i++) {

        xa = xa + myIMU.readRawAccelX();
        ya = ya + myIMU.readRawAccelY();
        za = za + myIMU.readRawAccelZ();
    }

    // ----------- Gyro offset
    for (int i = 0; i < iterations; i++) {

        xg = xg + myIMU.readRawGyroX();
        yg = yg + myIMU.readRawGyroY();
        zg = zg + myIMU.readRawGyroZ();
    }

    accOffsetX  = xa / iterations;
    accOffsetY  = ya / iterations;
    accOffsetZ  = za / iterations;  // - 1;
    gyroOffsetX = xg / iterations;
    gyroOffsetY = yg / iterations;
    gyroOffsetZ = zg / iterations;

    Serial.print(" accOffsetX: ");
    Serial.print(accOffsetX, 4);
    Serial.print(" accOffsetY: ");
    Serial.print(accOffsetY, 4);
    Serial.print(" accOffsetZ: ");
    Serial.print(accOffsetZ, 4);
    Serial.println();
    // ------------------
    Serial.print("*** Gyro offSet ***");
    Serial.print(" gyroOffsetX: ");
    Serial.print(gyroOffsetX, 4);
    Serial.print(" gyroOffsetY: ");
    Serial.print(gyroOffsetY, 4);
    Serial.print(" gyroOffsetZ: ");
    Serial.print(gyroOffsetZ, 4);
    Serial.println();
}

void calculatePR() {
    double x_Buff = float(accX);
    double y_Buff = float(accY);
    double z_Buff = float(accZ);
    aRoll         = atan2(y_Buff, z_Buff) * 57.295779513082321;
    aPitch        = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.295779513082321;
}