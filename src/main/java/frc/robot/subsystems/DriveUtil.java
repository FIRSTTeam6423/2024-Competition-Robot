// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.commons.Utility;

public class DriveUtil extends SubsystemBase {
	// P denotes Pivoting, D driving
	private final Translation2d m_frontLeftLoc = new Translation2d(Constants.TOPLEFT_X, Constants.TOPLEFT_Y);
	private final Translation2d m_frontRightLoc = new Translation2d(Constants.TOPRIGHT_X, Constants.TOPRIGHT_Y);
	private final Translation2d m_backLeftLoc = new Translation2d(Constants.BOTTOMLEFT_X, Constants.TOPLEFT_Y);
	private final Translation2d m_backRightLoc = new Translation2d(Constants.BOTTOMRIGHT_X, Constants.BOTTOMRIGHT_Y);
	private boolean started = false;
	private Field2d f2d = new Field2d();
	private AHRS gyro = new AHRS();

	private final SwerveModule m_frontLeft = new SwerveModule(
			Constants.FRONTLEFT_DRIVE, 
			true,
			Constants.FRONTLEFT_PIVOT,
			Constants.TOPLEFT_ABS_ENCODER, true);
	private final SwerveModule m_frontRight = new SwerveModule(
			Constants.FRONTRIGHT_DRIVE,
			true,
			Constants.FRONTRIGHT_PIVOT,
			Constants.TOPRIGHT_ABS_ENCODER, true);
	private final SwerveModule m_backLeft = new SwerveModule(
			Constants.BACKLEFT_DRIVE,
			true,
			Constants.BACKLEFT_PIVOT,
			Constants.BOTTOMLEFT_ABS_ENCODER, true);
	private final SwerveModule m_backRight = new SwerveModule(
			Constants.BACKRIGHT_DRIVE,
			true,
			Constants.BACKRIGHT_PIVOT,
			Constants.BOTTOMRIGHT_ABS_ENCODER, true);

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc,
			m_backLeftLoc, m_backRightLoc);

	// getPosition is just placeholder for getting distance with encoders even
	// though wpilib uses it as an example
	// this took me like 30 min ot figure out
	// convert encoders to m

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, getHeading2d(),
			new SwerveModulePosition[] {
					m_frontLeft.getPosition(),
					m_frontRight.getPosition(),
					m_backLeft.getPosition(),
					m_backRight.getPosition()
			}, new Pose2d(0.0, 0.0, new Rotation2d()));

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		setSwerveModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public void setSwerveModuleStates(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_LINEAR_SPEED);

		m_frontLeft.setDesiredState(states[0]);
		m_frontRight.setDesiredState(states[1]);
		m_backLeft.setDesiredState(states[2]);
		m_backRight.setDesiredState(states[3]);
	}

	public Rotation2d getHeading2d() {
		return gyro.getRotation2d();
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public double getYaw() {
		return gyro.getYaw();
	}

	public double getPitch(){
		return gyro.getPitch();
	}

	public double getRoll(){
		return gyro.getRoll();
	}

	public void resetGyro() {
		gyro.reset();
	}

	public void resetPose(Pose2d pose) {
		m_odometry.resetPosition(getHeading2d(), new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
		}, pose);
	}

	public void flipOrientation(){
		Pose2d p=getPose();
		resetPose(new Pose2d(p.getTranslation(), p.getRotation().plus(Rotation2d.fromDegrees(180))));
	}

	public void calibrateGyro() {
		gyro.reset();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		m_odometry.update(getHeading2d(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_backLeft.getPosition(), m_backRight.getPosition()
				});
		
		f2d.setRobotPose(getPose());
		SmartDashboard.putData(f2d);
	}
}