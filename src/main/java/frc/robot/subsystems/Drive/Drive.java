package frc.robot.subsystems.Drive;

import static frc.robot.Constants.MAX_LINEAR_SPEED;
import static frc.robot.Constants.DriveConstants.ABS_ENCODER_OFFSETS;
import static frc.robot.Constants.DriveConstants.BACKLEFT_ABS_ENCODER;
import static frc.robot.Constants.DriveConstants.BACKLEFT_DRIVE;
import static frc.robot.Constants.DriveConstants.BACKLEFT_PIVOT;
import static frc.robot.Constants.DriveConstants.BACKRIGHT_ABS_ENCODER;
import static frc.robot.Constants.DriveConstants.BACKRIGHT_DRIVE;
import static frc.robot.Constants.DriveConstants.BACKRIGHT_PIVOT;
import static frc.robot.Constants.DriveConstants.FRONTLEFT_DRIVE;
import static frc.robot.Constants.DriveConstants.FRONTLEFT_PIVOT;
import static frc.robot.Constants.DriveConstants.FRONTRIGHT_ABS_ENCODER;
import static frc.robot.Constants.DriveConstants.FRONTRIGHT_DRIVE;
import static frc.robot.Constants.DriveConstants.FRONTRIGHT_PIVOT;
import static frc.robot.Constants.DriveConstants.m_backLeftLoc;
import static frc.robot.Constants.DriveConstants.m_backRightLoc;
import static frc.robot.Constants.DriveConstants.m_frontLeftLoc;
import static frc.robot.Constants.DriveConstants.m_frontRightLoc;
import static frc.robot.Constants.DriveConstants.m_offset;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.Gyro.GyroIO;
import frc.robot.subsystems.Drive.Gyro.GyroSimIO;
import frc.robot.subsystems.Drive.Gyro.NavxIO;
import frc.robot.subsystems.Drive.Module.ModuleIO;
import frc.robot.subsystems.Drive.Module.NeoCoaxialModule;
import frc.robot.subsystems.Drive.Module.SimModule;
import java.util.List;
import java.util.function.Supplier;

public class Drive extends SubsystemBase {

  private final ModuleIO frontLeft;
  private final ModuleIO frontRight;
  private final ModuleIO backLeft;
  private final ModuleIO backRight;

  private final List<ModuleIO> swerveModules;
  private final GyroIO gyroIO;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;

  private static Rotation2d simRotation = new Rotation2d();
  private final Field2d f2d;
  private final FieldObject2d[] s2d;
  StructArrayPublisher<SwerveModuleState> swervePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
          .publish();

  public Drive() {
    frontLeft =
        Robot.isReal()
            ? new NeoCoaxialModule(
                "frontLeft",
                ABS_ENCODER_OFFSETS[0],
                FRONTLEFT_PIVOT,
                FRONTLEFT_DRIVE,
                0,
                false,
                true)
            : new SimModule("frontLeft");
    frontRight =
        Robot.isReal()
            ? new NeoCoaxialModule(
                "frontRight",
                ABS_ENCODER_OFFSETS[1],
                FRONTRIGHT_PIVOT,
                FRONTRIGHT_DRIVE,
                FRONTRIGHT_ABS_ENCODER,
                true,
                true)
            : new SimModule("frontRight");
    backLeft =
        Robot.isReal()
            ? new NeoCoaxialModule(
                "backLeft",
                ABS_ENCODER_OFFSETS[2],
                BACKLEFT_PIVOT,
                BACKLEFT_DRIVE,
                BACKLEFT_ABS_ENCODER,
                true,
                true)
            : new SimModule("backLeft");
    backRight =
        Robot.isReal()
            ? new NeoCoaxialModule(
                "backRight",
                ABS_ENCODER_OFFSETS[3],
                BACKRIGHT_PIVOT,
                BACKRIGHT_DRIVE,
                BACKRIGHT_ABS_ENCODER,
                false,
                true)
            : new SimModule("backRight");

    swerveModules = List.of(frontLeft, frontRight, backLeft, backRight);
    gyroIO = Robot.isReal() ? new NavxIO() : new GyroSimIO();

    zeroHeading();
    kinematics =
        new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);
    odometry =
        new SwerveDrivePoseEstimator(
            kinematics, getGyroHeading(), getModulePositions(), new Pose2d());

    f2d = new Field2d();
    s2d = new FieldObject2d[swerveModules.size()];
    for (int i = 0; i < swerveModules.size(); i++) {
      var module = swerveModules.get(i);
      s2d[i] = f2d.getObject("swerve module-" + module.getModuleID());
    }
  }

  // * Control logic
  /** Zero out gyro */
  public void zeroHeading() {
    gyroIO.resetGyro();
  }

  /**
   * Gets gyro orientation
   *
   * @return Rotation2d
   */
  public Rotation2d getGyroHeading() {
    return gyroIO.getGyroHeading();
  }

  /**
   * Gets each swerve module position
   *
   * @return SwerveModulePosition[]
   */
  public SwerveModulePosition[] getModulePositions() {
    var modulePositions = new SwerveModulePosition[swerveModules.size()];
    for (int i = 0; i < swerveModules.size(); i++) {
      modulePositions[i] = swerveModules.get(i).getModulePosition();
    }
    return modulePositions;
  }

  /**
   * Gets each swerve module state
   *
   * @return SwerveModuleState[]
   */
  public SwerveModuleState[] getModuleStates() {
    var moduleStates = new SwerveModuleState[swerveModules.size()];
    for (int i = 0; i < swerveModules.size(); i++) {
      moduleStates[i] = swerveModules.get(i).getModuleState();
    }
    return moduleStates;
  }

  /**
   * Sets each swerve module's state
   *
   * @param states desired states
   */
  public void setDesiredStates(SwerveModuleState[] states) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED); // !!!!!!!!!!! add to constants later

    for (int i = 0; i < swerveModules.size(); i++) {
      swerveModules.get(i).updateDesiredState(states[i]);
    }
  }

  /** Stops all drive and pivot motors */
  public void stopModules() {
    for (int i = 0; i < swerveModules.size(); i++) {
      swerveModules.get(i).setDriveVoltage(0);
      swerveModules.get(i).setPivotVoltage(0);
    }
  }

  // * public interface
  /**
   * Gets odometry's pose estimate
   *
   * @return Pose2d
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Gets chassis speed
   *
   * @return {@link ChassisSpeeds}
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Resets odometry
   *
   * @param pose Position to reset to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Robot.isReal() ? getGyroHeading() : simRotation, getModulePositions(), pose);
  }

  public void updateOdometry(SwerveModulePosition[] modulePositions) {
    odometry.update(Robot.isReal() ? getGyroHeading() : simRotation, modulePositions);
  }

  /**
   * Drives robot based on provided {@link ChassisSpeeds}
   *
   * @param chassisSpeeds {@link ChassisSpeeds}
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setDesiredStates(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /**
   * Drives robot based on velocities
   *
   * @param vxMPS X translation velocity (m/s)
   * @param vyMPS Y translation velocity (m/s)
   * @param omegaRPS Rotational velocity (rads/s)
   */
  public Command driveRobot(Supplier<ChassisSpeeds> chassisSpeeds) {
    return this.runOnce(
        () -> {
          //ChassisSpeeds speeds = new ChassisSpeeds(vxMPS, vyMPS, omegaRPS);
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  chassisSpeeds.get().vxMetersPerSecond,
                  chassisSpeeds.get().vyMetersPerSecond,
                  chassisSpeeds.get().omegaRadiansPerSecond,
                  getPose().getRotation());

          var allianceSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  DriverStation.getAlliance().get() == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          speeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02); // !!!!!!!!!!!!
          setChassisSpeeds(speeds);
        });
  }

  /** Stops the robot */
  public void stopRobot() {
    setChassisSpeeds(new ChassisSpeeds());
  }

  @Override
  public void periodic() {
    super.periodic();
    updateOdometry(getModulePositions());
    f2d.setRobotPose(getPose());
    for (int i = 0; i < s2d.length; i++) {
      var module = swerveModules.get(i);
      var transform = new Transform2d(m_offset[i], module.getModulePosition().angle);
      s2d[i].setPose(getPose().transformBy(transform));
    }
    SmartDashboard.putData(f2d);
    simRotation =
        simRotation.rotateBy(
            Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * 0.020));
    swervePublisher.set(getModuleStates());
  }
}
