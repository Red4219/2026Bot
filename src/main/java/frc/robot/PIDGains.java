// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 *  Class that organizes gains used when assigning values to slots
 */
public class PIDGains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	
	public PIDGains(double kP, double kI, double kD) {
		this(kP, kI, kD, 0, 0, 0);
	}

	public PIDGains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.kIzone = kIzone;
		this.kPeakOutput = kPeakOutput;
	}

}