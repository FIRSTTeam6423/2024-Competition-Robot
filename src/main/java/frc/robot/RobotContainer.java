// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AmpMech.AmpMech;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private static XboxController driver = new XboxController(0);
  private static XboxController operator = new XboxController(1);
  private static CommandXboxController operatorCommandController = new CommandXboxController(1);
  private static CommandXboxController driverCommandController = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  //private CargoUtil cargoUtil = new CargoUtil();

  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private AmpMech ampMech = new AmpMech();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //autoChooser.setDefaultOption("Run Shooter", new ShooterRollerTest(cargoUtil));
    // Configure the trigger bindings
    configureDefaultCommands();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
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
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    operatorCommandController.y().onTrue(ampMech.grabNote());
    
  }

  private void configureDefaultCommands() {
    //cargoUtil.setDefaultCommand(new IntakePivotTest(cargoUtil));
  }

  public static boolean getDriverIntakeInput() {
    return driver.getRightTriggerAxis() >= .5;
  }

  public static boolean getDriverFireButton() {
    return driver.getRightBumper();
  }

  //TODO ADD ACTUAL BUTTON AND USE OPERATOR CONTROLLER INTSEAD
  // public static boolean getOperatorSpinupInput() {      
  //   return operator.getRightBumper();
  // }

  // public static void rumbleOperator(GenericHID.RumbleType rmb, double n) {
  //   operatorCommandController.setRumble(rmb, n);
  // }

  public static Command rumbleDriverCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(()->operator.setRumble(rmb, n));
  }

  public static Command rumbleOperatorCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(()->driver.setRumble(rmb, n));
  }

  public static boolean getOperatorHandoffInput(){
    return driver.getAButton();
  }

  //TODO ADD ACTUAL BUTTON AND USE OPERATOR CONTROLLER INTSEAD
  public static boolean getOperatorStowManualOverrideInput() {
    return driver.getAButton();
  }

  //TODO ADD ACTUAL BUTTON
  public static boolean getDriverBButton(){
    return driver.getBButton();
  }

  public static boolean getDriverShootInput() {
    return driver.getRightTriggerAxis() > .5;
  }
  //TODO ADD ACTUAL BUTTON
  public static boolean getDriverDepositInput(){
    return driver.getLeftTriggerAxis() > .5;
  }

  //public Command getAutonomousCommand(){
    //return new ShooterRollerTest(cargoUtil);
  //}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
