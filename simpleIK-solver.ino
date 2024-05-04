#include <Servo.h>
#include <math.h>


Servo armServo1;
Servo armServo2;

int pos1 = 0;
int pos2 = 0;

const double L1 = 10.0; // Length of link 1
const double L2 = 7.0;  // Length of link 2

double IK_x, IK_y;

double IK(double x, double y) {
    double r = sqrt(x * x + y * y);

    // Check if the end effector coordinates are reachable
    if (r > L1 + L2) {
        Serial.println("End effector coordinates are outside the reachable workspace");
        return 0.0;
    }

    double phi = atan2(y, x);

    double alpha = acos(min((L1 * L1 + r * r - L2 * L2) / (2 * L1 * r), 1.0));
    double theta1 = phi - alpha;

    double beta = acos(min((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2), 1.0));
    double theta2 = M_PI - beta;

    return theta1;
}

double performIK(double x, double y) {
    return IK(x, y);
}

void setup() {
    armServo1.attach(9); // Attach servo 1 to pin 9
    armServo2.attach(10); // Attach servo 2 to pin 10

    Serial.begin(9600); // Initialize serial communication
}

void loop() {
    if (Serial.available() > 0) {
        String userInput = Serial.readStringUntil('\n');

        if (userInput.substring(0, 5) == "point") {
            double x, y;
            // Read coordinates
            if (sscanf(userInput.c_str(), "point (%lf,%lf)", &x, &y) == 2) {
                double angle = performIK(x, y);
                armServo1.write(angle);
                armServo2.write(angle);
            }
        }
    }
}
