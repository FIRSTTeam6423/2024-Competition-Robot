// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SimModule extends SubsystemBase {
	/** Creates a new SwerveModule. */
	private DCMotorSim driveMotor = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(2.0928, 0.12098),
          DCMotor.getNEO(1),
          Constants.DRIVE_GEAR_RATIO);
  private DCMotorSim pivotMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(0.22, 0.012),
    DCMotor.getNEO(1),
    1);
	// private RelativeEncoder driveEncoder;
	// private DutyCycleEncoder pivotEncoder;

	// Gains are for example purposes only - must be determined for your own robot!
	private PIDController drivePIDController, pivotPIDController;

	private int encoderID;

	private SwerveModuleState state;

	public SimModule(int driveMotorID, boolean driveInverted, int pivotMotorID, int encoderID, boolean pivotInverted) {
		this.encoderID = encoderID;
		/**
		 * We need three encoders, as the sparkmax can only accurately tell
		 * the rpm of the motor, not its position. A third ecnoder needs to handle
		 * that and pass that in to the PIDController
		 **/
		// driveEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_METERS);
		// driveEncoder.setVelocityConversionFactor(Constants.RPM_TO_METERS_PER_SEC);
		// driveEncoder.setPosition(0);

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

  public Rotation2d getPivotRotation() {
    return Rotation2d.fromDegrees(Math.toDegrees(pivotMotor.getAngularPositionRad())); //- Constants.ABS_ENCODER_OFFSETS[this.encoderID]);
  }

  public double getDriveVelocity() {
    return driveMotor.getAngularVelocityRPM() * Constants.RPM_TO_METERS_PER_SEC;
  }

  public double getDrivePositionMeters() {
    return driveMotor.getAngularPositionRotations() * Constants.DRIVE_ROTATIONS_TO_METERS;
  }

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePositionMeters(), getPivotRotation());
	}

  public SwerveModuleState getState() {
		return new SwerveModuleState(
      getDriveVelocity(), 
      getPivotRotation()
    );
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		double curRotDeg = getPivotRotation().getDegrees();

		state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(curRotDeg));
		// Different constant need for drivePIDController, convert m/s to rpm
		driveMotor.setInput(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
		pivotMotor.setInput(pivotPIDController.calculate(curRotDeg, state.angle.getDegrees()));
		SmartDashboard.putNumber("DRIVE VEL", getDriveVelocity());
	}

	public void stopModule() {
		driveMotor.setInput(0);
		pivotMotor.setInput(0);
	}

	public void resetEncoders() {
		driveMotor.setState(VecBuilder.fill(0, 0));
    pivotMotor.setState(VecBuilder.fill(0, 0));
	}

	@Override
	public void periodic() {

	}
}