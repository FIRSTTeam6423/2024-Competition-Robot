// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LockOntoNote;
import frc.robot.commands.OperateDrive;
import frc.robot.commons.VisionUpdate;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.VisionUtil;

import frc.robot.Climb.Climb;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
 // private static final VisionUtil visionUtil = new VisionUtil();
  private static final DriveUtil driveUtil = new DriveUtil();
  private static final VisionUtil visionUtil = new VisionUtil();

  private static XboxController driver = new XboxController(0);
  private static XboxController operator = new XboxController(1);
  private static CommandXboxController operatorCommandController = new CommandXboxController(1);
  private static CommandXboxController driverCommandController = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

	private static final PhotonCamera aprilCamFront = new PhotonCamera("aprilcamfront");
	private static final PhotonCamera aprilCamBack = new PhotonCamera("aprilcamback");

  public static double allianceOrientation = 0;

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  public static Pose3d getTagPose3dFromId(int id) {
    return Constants.TagPoses[id - 1];
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverCommandController.leftBumper().onTrue(new LockOntoNote(driveUtil));
    operatorCommandController.rightStick().onTrue(new Climb().OperateClimb()).onFalse(new Climb().StopClimb());
    operatorCommandController.leftStick().onTrue(new Climb().OperateClimb()).onFalse(new Climb().StopClimb());
  }

  private void configureDefaultCommands() {
    //driveUtil.setDefaultCommand(getAutonomousCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static double getDriverLeftXboxX() {
    return driver.getLeftX();
  }

  public static double getDriverLeftXboxY() {
    return driver.getLeftY();
  }

  public static double getDriverRightXboxX() {
    return driver.getRightX();
  }

  public static double getDriverRightXboxY() {
    return driver.getRightY();
  }

  public static double getDriverLeftXboxTrigger() {
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightXboxTrigger() {
    return driver.getRightTriggerAxis();
  }

  public static boolean getDriverLeftBumper() {
    return driver.getLeftBumper();
  }
  
 public static double getOperatorLeftXboxX() {
    return operator.getLeftX();
  }

  public static double getOperatorLeftXboxY() {
    return operator.getLeftY();
  }

  public static double getOperatorRightXboxX() {
    return operator.getRightX();
  }

  public static double getOperatorRightXboxY() {
    return operator.getRightY();
  }

  public static double getOperatorLeftXboxTrigger() {
    return operator.getLeftTriggerAxis();
  }

  public static double getOperatorRightXboxTrigger() {
    return operator.getRightTriggerAxis();
  }

  public static boolean getOperatorLeftBumper() {
    return operator.getLeftBumper();
  }

  // Gets the robot's position from the nearest april tag
  public static List<VisionUpdate> getVisionPoseUpdatesMeters() {
    return visionUtil.getVisionPoseUpdatesMeters();
  }

}
