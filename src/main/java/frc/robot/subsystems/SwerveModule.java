// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */
	private CANSparkMax driveMotor, pivotMotor;

	private RelativeEncoder driveEncoder;
	private DutyCycleEncoder pivotEncoder;

	// Gains are for example purposes only - must be determined for your own robot!
	private PIDController drivePIDController, pivotPIDController;

	private int encoderID;

	private SwerveModuleState state;

	public SwerveModule(int driveMotorID, boolean driveInverted, int pivotMotorID, int encoderID, boolean pivotInverted) {
		driveMotor.setInverted(driveInverted);
		pivotMotor.setInverted(pivotInverted);
		this.encoderID = encoderID;
		/**
		 * We need three encoders, as the sparkmax can only accurately tell
		 * the rpm of the motor, not its position. A third ecnoder needs to handle
		 * that and pass that in to the PIDController
		 **/
		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(Constants.DRIVECONVERSIONFACTOR);
		driveEncoder.setVelocityConversionFactor(Constants.DRIVECONVERSIONFACTOR/60);
		driveEncoder.setPosition(0);

		pivotEncoder = new DutyCycleEncoder(encoderID);
		pivotEncoder.reset();

		drivePIDController = new PIDController(Constants.MODULEDRIVE_P, Constants.MODULEDRIVE_I, Constants.MODULEDRIVE_D);
		drivePIDController.setP(Constants.MODULEDRIVE_P);
		drivePIDController.setI(Constants.MODULEDRIVE_I);
		drivePIDController.setD(Constants.MODULEDRIVE_D);

		pivotPIDController = new PIDController(Constants.MODULEPIVOT_P, Constants.MODULEPIVOT_I, Constants.MODULEPIVOT_D);
		pivotPIDController.enableContinuousInput(-90,90);
		pivotPIDController.setP(Constants.MODULEPIVOT_P);
		pivotPIDController.setI(Constants.MODULEPIVOT_I);
		pivotPIDController.setD(Constants.MODULEPIVOT_D);

		state = getState();
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(pivotEncoder.getAbsolutePosition() * Constants.DEGREES_PER_ROTATION - Constants.ABS_ENCODER_OFFSETS[this.encoderID]));
	}

	public SwerveModulePosition getPosition() {
		Rotation2d r = Rotation2d.fromDegrees(pivotEncoder.getAbsolutePosition() * Constants.DEGREES_PER_ROTATION - Constants.ABS_ENCODER_OFFSETS[this.encoderID]);
		return new SwerveModulePosition(driveEncoder.getPosition(),
				r);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		double curRotDeg = pivotEncoder.getAbsolutePosition() * Constants.DEGREES_PER_ROTATION - Constants.ABS_ENCODER_OFFSETS[this.encoderID];//-pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID];
		//if(this.encoderID == 7) {
		//}"
		state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(curRotDeg));
		// Different constant need for drivePIDController, convert m/s to rpm
		driveMotor.set(drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond));
		pivotMotor.set(pivotPIDController.calculate(curRotDeg, state.angle.getDegrees()));
	}

	public void stopModule() {
		driveMotor.set(0);
		pivotMotor.set(0);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		pivotEncoder.reset();
	}

	public double pivotValue(){
		return pivotEncoder.getAbsolutePosition();
	}

	public double drivePosValue(){
		return driveEncoder.getPosition();
	}

	public double driverVelValue(){
		return driveEncoder.getVelocity();
	}

	public double stateSpeed(){
		return state.speedMetersPerSecond;
	}

	public double stateAngle(){
		return state.angle.getDegrees();
	}

	@Override
	public void periodic() {

	}
}