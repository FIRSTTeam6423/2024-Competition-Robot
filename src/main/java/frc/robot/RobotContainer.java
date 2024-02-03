// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LockOntoNote;
import frc.robot.commons.VisionUpdate;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.VisionUtil;

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
  private static final VisionUtil visionUtil = new VisionUtil();
  private static final DriveUtil driveUtil = new DriveUtil();

	private static final PhotonCamera aprilCamFront = new PhotonCamera("aprilcamfront");
	private static final PhotonCamera aprilCamBack = new PhotonCamera("aprilcamback");

  public static double allianceOrientation = 0;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private static CommandXboxController driverCommandController;
  private static XboxController driver;

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new XboxController(Constants.XBOX_DRIVER);
    // Configure the trigger bindings
    driver = new XboxController(Constants.XBOX_DRIVER);
    driverCommandController = new CommandXboxController(Constants.XBOX_DRIVER);
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

  // Gets the robot's position from the nearest april tag
  public static List<VisionUpdate> getVisionPoseUpdatesMeters() {
    return visionUtil.getVisionPoseUpdatesMeters();
  }

}
