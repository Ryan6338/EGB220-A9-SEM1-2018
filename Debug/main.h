#ifndef MAIN_H
#define MAIN_H

#define MAX_SPEED 1
#define COUNTDOWN_TIME .5
#define TURN_FILTER 0.05
#define SLIGHT_TURN 0.3
#define HARD_TURN 0.6

typedef enum RobotState {
	RUNNING,
	COUNTDOWN,
	STOPPED
} RobotState;

#endif