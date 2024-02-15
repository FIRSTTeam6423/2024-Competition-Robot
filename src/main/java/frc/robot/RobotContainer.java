// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.CargoUtil;
import frc.robot.Subsystems.Intake;
import frc.robot.commands.AmpMechPivotTest;
import frc.robot.commands.HandleUserCargoInput;
import frc.robot.commands.IntakePivotTest;
import frc.robot.commands.ShooterRollerTest;
import frc.robot.commands.HandleUserCargoInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
  private static CommandXboxController driverCommandController;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private CargoUtil cargoUtil = new CargoUtil();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("Run Shooter", new ShooterRollerTest(cargoUtil));
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
    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5).and(()-> !Intake.hasNote())
      .whileTrue(Intake.extend())
        .andThen(Intake.retract()); 
    
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  private void configureDefaultCommands() {
    cargoUtil.setDefaultCommand(new IntakePivotTest(cargoUtil));
  }

  public static boolean getDriverIntakeInput() {
    return driver.getRightTriggerAxis() >= .5;
  }

  public static boolean getDriverFireButton() {
    return driver.getRightBumper();
  }

  //TODO ADD ACTUAL BUTTON AND USE OPERATOR CONTROLLER INTSEAD
  public static boolean getOperatorSpinupInput() {      
    return operator.getRightBumper();
  }

  public static void rumbleOperator(GenericHID.RumbleType rmb, double n) {
    operator.setRumble(rmb, n);
  }

  public static void rumbleDriver(GenericHID.RumbleType rmb, double n) {
    driver.setRumble(rmb, n);
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
