#include <iostream>
#include <cmath>

void printPair(const std::pair<double, double>& p) {
    std::cout << "(" << p.first << ", " << p.second << ")";
}

const double L1 = 10.0; // Length of link 1
const double L2 = 7.0;  // Length of link 2

std::pair<double, double> IK(double x, double y) {
    double r = sqrt(x*x + y*y);

    // Check if the end effector coordinates are reachable
    if (r > L1 + L2) {
        std::cerr << "End effector coordinates are outside the reachable workspace\n";
        return std::make_pair(0.0, 0.0);
    }

    double phi = atan2(y, x);

    double alpha = acos(std::min((L1*L1 + r*r - L2*L2) /     (2*L1*r), 1.0));
    double theta1 = phi - alpha;

    double beta = acos(std::min((L1*L1 + L2*L2 - r*r) / (2*L1*L2), 1.0));
    double theta2 = M_PI - beta;

    return std::make_pair(theta1, theta2);
}

void performIK(double x, double y) {
    auto result = IK(x, y); // Assigning IK(x,y) to a variable result

    double angle1 = result.first;
    double angle2 = result.second;

    // Convert radians to degrees
    double angle1_deg = angle1 * (180.0 / M_PI);
    double angle2_deg = angle2 * (180.0 / M_PI);

    std::cout << "Joint Angle 1: " << angle1_deg << " degrees\n";
    std::cout << "Joint Angle 2: " << angle2_deg << " degrees\n";
}

int main() {
    double x = 4.0; // End effector x-coordinate
    double y = 1.0; // End effector y-coordinate

    performIK(x, y);

    return 0;
}
