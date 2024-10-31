// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AmpMech.AmpMech;
import frc.robot.Climb.Climb;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Shooter.Shooter;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Drive.Drive;
import frc.robot.Drive.DriveConstants;

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

  private static Joystick joystick = new Joystick(2);
  private static CommandXboxController operatorCommandController = new CommandXboxController(1);
  private static CommandXboxController driverCommandController = new CommandXboxController(0);
  private static CommandJoystick joystickController = new CommandJoystick(2);

  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private AmpMech ampMech = new AmpMech();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();

  public static boolean allowDeposit;

  private GenericEntry intakeVoltEntry = Shuffleboard.getTab("Intake").add("Pivot Volts", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  private PIDController lockRotationController = new PIDController(.021, 0, .001);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    intake.retract().schedule();
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

    //INTAKE CONTROL
    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
        .onTrue(intake.startIntake())
        .onFalse(intake.retract()
      );
 
    operatorCommandController.povUp().whileTrue(intake.startOutake()).onFalse(intake.retract());

    joystickController.button(Joystick.ButtonType.kTrigger.value).onTrue(intake.outakeRolling(1)).onFalse(intake.stopRoller());

    joystickController.povDown().whileTrue(intake.setVoltsRamp(IntakeConstants.ROLLER_INTAKE_SPEED)).onFalse(intake.stopRoller());
    
    //joystickController.axisGreaterThan(Joystick.AxisType.kY.value, .1).whileTrue(
      //intake.setSetpoint(intake.getAngleRelativeToGround().plus(Rotation2d.fromDegrees(joystickController.getRawAxis(Joystick.AxisType.kY.value))))
    //);

    //joystickController.axisLessThan(Joystick.AxisType.kY.value, -.1).whileTrue(
      //intake.setSetpoint(intake.getAngleRelativeToGround().plus(Rotation2d.fromDegrees(joystickController.getRawAxis(Joystick.AxisType.kY.value)).times(0.5)))
    //);

    joystickController.axisGreaterThan(Joystick.AxisType.kY.value, .2).whileTrue(intake.betterCommand(//intake.getAngleRelativeToGround().getDegrees()));
    ));

    joystickController.axisLessThan(Joystick.AxisType.kY.value, -.2).whileTrue(intake.betterCommand2(//intake.getAngleRelativeToGround().getDegrees()));
    ));
  }
  
  public void registerAutoCommands() {

  }

  private void configureDefaultCommands() {

    // intake.setDefaultCommand(
    //   spitCannon(
    //     Rotation2d.fromDegrees(
    //       joystickController.getRawAxis(Joystick.AxisType.kY.value) * 3
    //   );
    // );

  }

  public Command spitCannon(Rotation2d angle) {

    return Commands.run(() -> {
      if (intake.getAngleRelativeToGround().getDegrees() <= IntakeConstants.PIVOT_OUT_ANGLE && intake.getAngleRelativeToGround().getDegrees() >= IntakeConstants.PIVOT_IN_ANGLE) {
        intake.setGoal(intake.getAngleRelativeToGround().getDegrees() + angle.getDegrees());
      }
    });

  }

}
