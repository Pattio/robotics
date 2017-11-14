#include <math.h>


enum FacingDirection { east, north, south, west };


FacingDirection getDirection(double angle) {
	if((angle >= 0 && angle <= M_PI / 4) || (angle < 0 && angle >= -M_PI / 4)) {
		return FacingDirection::east;
	} else if((angle > M_PI / 4 && angle <= M_PI / 2) || (angle >= M_PI / 2 && angle < M_PI * 3 / 4)) {
		return FacingDirection::north;
	} else if((angle < -M_PI / 4 && angle >= -M_PI / 2) || (angle < -M_PI / 2 && angle > -M_PI * 3 / 4)) {
		return FacingDirection::south;
	} else {
		return FacingDirection::west;
	}
}


double round(double x) { return floor(x * 100 + 0.5) / 100; }
double angle(double currentX, double currentY, double targetX, double targetY) {
	return atan2(round(targetY) - round(currentY), round(targetX) - round(currentX));
}