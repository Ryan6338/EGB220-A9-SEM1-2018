
float line_edge_value;
float kP, kI, kD;


/* Convert sensor values to estimate angle from tangent of line
 * Can be modified to account for radius of line arc to improve smoothness but will be restricted by width of sensor array

float calculate_angle() {
	Convert sensor values to represent deviation in angle from straight
	
		Scale sensor input to minimum and maximum possible values between 0 and 1:
			Scaled_Value = (Sensor_N_Value - Minimum_Value) / Maximum_Value
	
		Convert scaled angle to estimated angle, discard any sensor values that are outside of expected value ranges as they provide no data:
			Angle_to_Sensor = toAngle(Scaled_Value)
			toAngle() function needs to be determined by measuring sensor outputs as they're moved across the line. May be non-linear
	
		Average known angles to sensors to get estimation of robot angle to line
			angle = (sensor_1 + dAngPivot_1 + ... + sensor_n + dAngPivot_n) / n;
	
	
	return 0f;
}*/

int32_t error_integral = 0;

void update_drive() {
	//Calculate angular error from sensor array. Target position is always 0deg so error is opposite of current angle (0 - current angle).
	int16_t error = calculate_angle();
	error_integral += error;
}