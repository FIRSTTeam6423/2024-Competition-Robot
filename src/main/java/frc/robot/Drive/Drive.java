// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Drive.SwerveModule;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drive extends SubsystemBase {
	private final Translation2d m_frontLeftLoc = new Translation2d(Constants.FRONTLEFT_X, Constants.FRONTLEFT_Y);
	private final Translation2d m_frontRightLoc = new Translation2d(Constants.FRONTRIGHT_X, Constants.FRONTRIGHT_Y);
	private final Translation2d m_backLeftLoc = new Translation2d(Constants.BACKLEFT_X, Constants.FRONTLEFT_Y);
	private final Translation2d m_backRightLoc = new Translation2d(Constants.BACKRIGHT_X, Constants.BACKRIGHT_Y);
	private Field2d f2d = new Field2d();
	private AHRS gyro = new AHRS();

	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;

	public Rotation2d simRotation = new Rotation2d();

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc,
			m_backLeftLoc, m_backRightLoc);

	// getPosition is just placeholder for getting distance with encoders even
	// though wpilib uses it as an example
	// this took me like 30 min ot figure out
	// convert encoders to m

	private final SwerveDriveOdometry m_odometry;

	public double deadzone(double input) {
		if (Math.abs(input) >= Constants.XBOX_STICK_DEADZONE_WIDTH) {
			return input;
		} else {
			return 0;
		}
	}

	public Rotation2d getHeading2d() {
		return gyro.getRotation2d();
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return kinematics.toChassisSpeeds(getSwerveModuleStates()); // m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc,
																	// m_backRightLoc
	}

	public Pose2d getAngleNegatedPose() {
		Pose2d p = getPose();
		return new Pose2d(p.getTranslation(), p.getRotation().times(-1));
	}

	public double getHeading() {
		return gyro.getYaw();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	public double getRoll() {
		return gyro.getRoll();
	}

	public SwerveModuleState[] getSwerveModuleStates() {
		// m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc
		SwerveModuleState[] states = { m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
				m_backRight.getState() };
		return states;
	}

	public void setSwerveModuleStates(SwerveModuleState[] states) {
		m_frontLeft.setDesiredState(states[0]);
		m_frontRight.setDesiredState(states[1]);
		m_backLeft.setDesiredState(states[2]);
		m_backRight.setDesiredState(states[3]);
	}

	public void resetPose(Pose2d pose) {
		m_odometry.resetPosition(getHeading2d(), new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_backLeft.getPosition(),
				m_backRight.getPosition()
		}, pose);
	}

	// -- COMANDS --

	public Command driveRobot(boolean fieldRelative) {
		return this.runOnce(() -> {
			int xSign = (int) Math.signum(RobotContainer.getDriverLeftXboxY());
			double xSpeed = xSign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxY()), 2)
					* Constants.MAX_LINEAR_SPEED
			// * Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); // reversed x and y so that up on
																						// controller is

			int ySign = (int) Math.signum(RobotContainer.getDriverLeftXboxX());
			double ySpeed = ySign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxX()), 2)
					* Constants.MAX_LINEAR_SPEED
			// * Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); // reversed x and y so that up on
																						// controller is

			double omega = deadzone(RobotContainer.getDriverRightXboxX())
					* Math.toRadians(Constants.MAX_ANGULAR_SPEED)
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1);

			var swerveModuleStates = kinematics.toSwerveModuleStates(
					fieldRelative
							? ChassisSpeeds.fromFieldRelativeSpeeds(
									xSpeed, // reversed x and y so that up on controller is
									ySpeed, // forward from driver pov
									omega,
									m_odometry.getPoseMeters().getRotation())
							: new ChassisSpeeds(RobotContainer.getDriverLeftXboxY() * Constants.MAX_LINEAR_SPEED,
									RobotContainer.getDriverLeftXboxX() * Constants.MAX_LINEAR_SPEED, // Note y and x
																										// swapped for
																										// first 2
																										// arguments is
																										// not
																										// intuitive, x
																										// is "forward"
									RobotContainer.getDriverRightXboxX() * Constants.MAX_ANGULAR_SPEED));

			// SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
			// Constants.MAX_LINEAR_SPEED);

			m_frontLeft.setDesiredState(swerveModuleStates[0]);
			m_frontRight.setDesiredState(swerveModuleStates[1]);
			m_backLeft.setDesiredState(swerveModuleStates[2]);
			m_backRight.setDesiredState(swerveModuleStates[3]);
		});
	}

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		setSwerveModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getSwerveModuleStates());
	}

	public Command flipOrientation() {
		return this.runOnce(() -> {
			Pose2d p = getPose();
			resetPose(new Pose2d(p.getTranslation(), p.getRotation().plus(Rotation2d.fromDegrees(180))));
		});
	}

	public void configureAutos() {
		AutoBuilder.configureHolonomic(
				this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
						4.5, // Max module speed, in m/s
						0.4, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
	}

	/** Creates a new Drive. */
	public Drive() {
		m_frontLeft = new SwerveModule(
				Constants.FRONTLEFT_DRIVE,
				true,
				Constants.FRONTLEFT_PIVOT,
				Constants.FRONTLEFT_ABS_ENCODER, true);
		m_frontRight = new SwerveModule(
				Constants.FRONTRIGHT_DRIVE,
				false,
				Constants.FRONTRIGHT_PIVOT,
				Constants.FRONTRIGHT_ABS_ENCODER, true);
		m_backLeft = new SwerveModule(
				Constants.BACKLEFT_DRIVE,
				false,
				Constants.BACKLEFT_PIVOT,
				Constants.BACKLEFT_ABS_ENCODER, true);
		m_backRight = new SwerveModule(
				Constants.BACKRIGHT_DRIVE,
				true,
				Constants.BACKRIGHT_PIVOT,
				Constants.BACKRIGHT_ABS_ENCODER, true);

		m_odometry = new SwerveDriveOdometry(kinematics, getHeading2d(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_backLeft.getPosition(),
						m_backRight.getPosition()
				}, new Pose2d(0.0, 0.0, new Rotation2d()));
		gyro.reset();
		// Configure pathplanner AutoBuilder
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("x pos", m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("y pos", m_odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("odo angle", getPose().getRotation().getDegrees());

		SmartDashboard.putNumber("pitch", gyro.getPitch());
		SmartDashboard.putNumber("yaw", gyro.getYaw());
		SmartDashboard.putNumber("roll", gyro.getRoll());

		SmartDashboard.putNumber("frontleft angle", m_frontLeft.getPosition().angle.getDegrees());
		SmartDashboard.putNumber("frontright angle", m_frontRight.getPosition().angle.getDegrees());
		SmartDashboard.putNumber("backleft angle", m_backLeft.getPosition().angle.getDegrees());
		SmartDashboard.putNumber("backright angle", m_backRight.getPosition().angle.getDegrees());

		m_odometry.update(Robot.isReal() ? getHeading2d() : simRotation,
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_backLeft.getPosition(), m_backRight.getPosition()
				});

		f2d.setRobotPose(getPose());
		SmartDashboard.putData(f2d);
	}

	@Override
	public void simulationPeriodic() {
		simRotation.rotateBy(
			Rotation2d.fromRadians(
				getChassisSpeeds().omegaRadiansPerSecond*(1/20)
			)
		);
	}
}