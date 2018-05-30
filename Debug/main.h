#ifndef MAIN_H
#define MAIN_H

#define COUNTDOWN_TIME .5

typedef enum RobotState {
	STOPPED,
	RUNNING_FULLSPEED,
	RUNNING_HALFSPEED,
	CALIBRATION,
	COUNTDOWN,
	FINISH
} RobotState;

#endif