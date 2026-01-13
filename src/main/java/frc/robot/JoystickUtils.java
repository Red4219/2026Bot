package frc.robot;

import frc.robot.Constants.OperatorConstants;

public class JoystickUtils {

	/**
	 * Applies a deadband to the input value from OperatorConstants
	 * 
	 * @param input the value to be constrained
	 * @return the constrained value
	 */
	public static double deadBand(double input) {
		if (Math.abs(input) < OperatorConstants.KDeadBand) {
			return 0;
		}
		return input;
	}

	/**
	 * squares the joystick input and uses clever math to compansate for the offset
	 * caused by the deadband
	 * 
	 * @param input The input from the joystick
	 * @return The corrected joystick values
	 */
	public static double processJoystickInput(double input) {

		// returns zero if input is less than deadband
		if (deadBand(input) == 0)
			return 0;

		double correctedValue = input;

		// does funky math to force linear input between deanband and 1
		correctedValue = (correctedValue - (OperatorConstants.KDeadBand * Math.signum(correctedValue)))
				/ (1 - OperatorConstants.KDeadBand);

		// raises input to a specified power for a smoother feel
		correctedValue = Math.copySign(Math.pow(Math.abs(correctedValue), OperatorConstants.kJoystickPow), input);

		return correctedValue;
	}

}