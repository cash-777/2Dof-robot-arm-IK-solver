#include <Servo.h>
#include <math.h>

// Servo name setup
Servo armServo1;
Servo armServo2;

const double L1 = 50; // Length of link 1
const double L2 = 50;  // Length of link 2

double IK_x, IK_y;

// Calculates IK
void IK(double x, double y, double &angle1, double &angle2) {
    double r = sqrt(x * x + y * y);

    // Check if the end effector coordinates are reachable
    if (r > L1 + L2) {
        Serial.println("End effector coordinates are outside the reachable workspace");
        return;
    }

    double phi = atan2(y, x);

    double alpha = acos(min((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r), 1.0));
    double theta1 = phi - alpha;

    double beta = acos(min((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2), 1.0));
    double theta2 = M_PI - beta;

    angle1 = (double)(theta1 * 180.0 / M_PI);
    angle2 = (double)(theta2 * 180.0 / M_PI);
}

void setup() {

  
    // Initialize serial communication
    Serial.begin(9600);
  //attach servo objects to desired pin (need to use a PWM pin)
    armServo1.attach(9);
    armServo2.attach(10);

    armServo1.write(180);
    armServo2.write(180);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command.startsWith("point(") && command.endsWith(")")) {
            // Extracting x and y coordinates from the command
            int commaIndex = command.indexOf(',');
            int endIndex = command.indexOf(')');
            if (commaIndex != -1 && endIndex != -1) {
                String x_str = command.substring(6, commaIndex);
                String y_str = command.substring(commaIndex + 1, endIndex);
                double x = x_str.toDouble();
                double y = y_str.toDouble();
                double servo1_angle, servo2_angle;
                IK(x, y, servo1_angle, servo2_angle);

               armServo1.write(servo1_angle);
               armServo2.write(servo2_angle);
            } else {
                Serial.println("Invalid command format");
            }
        } else {
            Serial.println("Invalid command");
        }
    }
}
