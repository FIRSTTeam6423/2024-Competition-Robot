// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AmpMech.AmpMech;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.OperateDrive;
import frc.robot.Drive.Drive;

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
  private static final Drive driveUtil = new Drive();

  private static XboxController driver = new XboxController(0);
  private static XboxController operator = new XboxController(1);
  private static CommandXboxController operatorCommandController = new CommandXboxController(1);
  private static CommandXboxController driverCommandController = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private AmpMech ampMech = new AmpMech();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureDefaultCommands();
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
    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5).and(()-> !intake.hasNote())
      .onTrue(intake.startIntake())
        .onFalse(intake.retract()); 
    
    driverCommandController.rightBumper().onTrue(intake.feed()).onFalse(intake.stopRoller());

    //if operator doesn't do spinup, shoot button will spinup anyway
    //if operator doesn't prime for amp deposit, amp release button on driver will NOT prime. WILL DO NOTHING

    operatorCommandController.rightBumper().whileTrue(
      shooter.spinup().alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 1))
      .until(()->driver.getRightBumper()).andThen(
        intake.feed().withTimeout(1).andThen(
          intake.stopRoller().asProxy().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        )
      )
    ).onFalse(shooter.stopRollers().alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 0)));

    operatorCommandController.b().onTrue(ampMech.extend());

    operatorCommandController.y().onTrue(ampMech.grabNote());
    
  }

  private void configureDefaultCommands() {
    driveUtil.setDefaultCommand(new OperateDrive(driveUtil));
  }

  public static Command rumbleDriverCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(()->operator.setRumble(rmb, n));
  }

  public static Command rumbleOperatorCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(()->driver.setRumble(rmb, n));
  }

  public static double getDriverLeftXboxY() {
    return driver.getLeftY();
  }

  public static double getDriverLeftXboxX() {
    return driver.getLeftX();
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
}
