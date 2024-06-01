package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.lib.IronUtil.*;
import static frc.robot.Constants.MAX_LINEAR_SPEED;
import static frc.robot.Constants.XBOX_STICK_DEADZONE_WIDTH;
import static frc.robot.Constants.XBOX_TRIGGER_DEADZONE_WIDTH;

import frc.robot.Constants.AmpMechConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.AmpMech.AmpMech;
import frc.robot.subsystems.AmpMech.AmpMechIOReal;
import frc.robot.subsystems.AmpMech.AmpMechIOSim;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbIOReal;
import frc.robot.subsystems.Climb.ClimbIOSim;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;

public class RobotContainer {
  // * ------ SUBSYSTEMS ------
  public final Drive drive;
  public final Climb climb;
  public final Intake intake;
  public final Shooter shooter;
  public final AmpMech ampMech;
  public final LEDSubsystem led;

  // * ------ AUTO (womp womp) ------
  public final SendableChooser<Command> autoSelector;

  // * ------ CONTROLLERS ------
  public static IronController driverController = new IronController(0, XBOX_STICK_DEADZONE_WIDTH, XBOX_TRIGGER_DEADZONE_WIDTH);
  public static IronController operatorController = new IronController(1, XBOX_STICK_DEADZONE_WIDTH, XBOX_TRIGGER_DEADZONE_WIDTH);

  // Contains subsystems
  public RobotContainer() {
    // Initalizes IO hardware for subsystems
    drive = new Drive();
    if (Robot.isReal()) {
      climb = new Climb(new ClimbIOReal());
      intake = Intake.getInstance();
      shooter = new Shooter(new ShooterIOReal());
      ampMech = new AmpMech(new AmpMechIOReal());
      led = new LEDSubsystem();
    } else {
      climb = new Climb(new ClimbIOSim());
      intake = Intake.getInstance();
      shooter = new Shooter(new ShooterIOSim());
      ampMech = new AmpMech(new AmpMechIOSim());
      led = new LEDSubsystem();
    }
    configureDefaultCommands();
    configureBindings();
    autoSelector = Autos.configureAutos(drive, intake, climb, ampMech, shooter);
    SmartDashboard.putData("Auto Chooser", autoSelector);
    intake.retract().schedule();
  }

  // ------ DEFAULT SUBSYSTEM COMMANDS ------
  /** These commands run when no other commands are running */
  private void configureDefaultCommands() {
    // x and y are swapped becausrobot's x is forward-backward, while controller x
    // is left-right
    drive.setDefaultCommand(
        drive.drive(
            () -> -driverController.joystickDeadbandOutput(0) * MAX_LINEAR_SPEED,
            () -> -driverController.joystickDeadbandOutput(1) * MAX_LINEAR_SPEED,
            () -> driverController.flickStickOutput(4, 3)
        )
        /*drive.drive(
            () -> new ChassisSpeeds(
                DriverStation.isAutonomous() 
                    ? 0 
                    : -driverController.joystickDeadbandOutput(0)
                        * Constants.MAX_LINEAR_SPEED,
                DriverStation.isAutonomous() 
                    ? 0 
                    : -driverController.joystickDeadbandOutput(1)
                        * Constants.MAX_LINEAR_SPEED,
                DriverStation.isAutonomous() 
                    ? 0 
                    : -driverController.joystickDeadbandOutput(4)
                        * Constants.MAX_ANGULAR_SPEED
            )
        )*/
    );
    new Trigger(DriverStation::isDisabled).whileTrue(led.enabledIdle());
    new Trigger(DriverStation::isEnabled).whileFalse(led.disabledIdle());

    // LED disabled idle mode
    led.setDefaultCommand(
        led.disabledIdle()
            .onlyWhile(DriverStation::isDisabled)
            .andThen(led.enabledIdle().onlyWhile(DriverStation::isEnabled)));
    led.disabledIdle().schedule();

    // Shooter Auto mode
    shooter.setDefaultCommand(
        shooter
            .setGoal(ShooterConstants.SHOOT_RPM)
            .onlyWhile(DriverStation::isAutonomous)
            .andThen(
                shooter.stopShooter().withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
  }

  private void configureBindings() {
    // * ------ TRIGGERS ------

    // White LED strobe when intake has note
    new Trigger(intake::hasNote)
        .onTrue(
            led.strobeLED(Color.kWhite, .1)
                .onlyIf(() -> intake.hasNote())
                .withTimeout(1.5)
                .andThen(led.setColor(Color.kBlack)));

    // Blue LED strobe when shooter reaches RPM
    new Trigger(shooter::atRPM)
        .onTrue(led.strobeLED(Color.kBlue, 0.05))
        .onFalse(led.setColor(Color.kBlack));

    // ---- DRIVER BINDS ----

    // -* RIGHT BUMPER TAP *- Feed note to shooter
    driverController
        .rightBumper()
        .onTrue(intake.shooterFeed())
        .onFalse(new WaitCommand(.75).onlyIf(shooter::atRPM).andThen(intake.stopRoller()));

    // -* LEFT BUMPER TAP *- Amp Control
    driverController
        .leftBumper()
        .onTrue(
            Commands.sequence(
                ampMech.setPivotGoal(AmpMechConstants.AMP_MECH_OUT_ANGLE),
                new WaitUntilCommand(ampMech::atGoal),
                ampMech.runRoller(AmpMechConstants.AMP_MECH_DEPOSIT_SPEED)));

    // -* RIGHT TRIGGER *- Intake control
    driverController
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
        .onTrue(intake.startIntake())
        .onFalse(intake.retract());
    
    // ---- OPERATOR BINDS ----

    // -* A BUTTON TAP *- LED green signal
    operatorController
        .a()
        .whileTrue(led.strobeLED(Color.kGreen, .25))
        .onFalse(led.setColor(Color.kBlack));

    // -* B BUTTON TAP *- LED red signal
    operatorController
        .b()
        .whileTrue(led.strobeLED(Color.kRed, .25))
        .onFalse(led.setColor(Color.kBlack));

    // -* X BUTTON TAP *- Unload amp
    operatorController
        .x()
        .and(() -> !intake.fullyHasNote())
        .onTrue(
            ampMech
                .runRoller(AmpMechConstants.SUCK_BACK_SPEED)
                .alongWith(intake.unload())
                .unless(() -> operatorController.getHID().getRightBumper()))
        .onFalse(intake.stopRoller().alongWith(ampMech.stopRollers()));

    // -* Y BUTTON TAP *- Amp handoff control
    operatorController
        .y()
        .onTrue(ampMech.setPivotGoal(AmpMechConstants.AMP_MECH_IN_ANGLE))
        .onFalse(
            Commands.sequence(
                new WaitUntilCommand(intake::atGoal),
                readyAmpMech().until(ampMech::beamBreakTriggered),
                new WaitUntilCommand(() -> !ampMech.beamBreakTriggered()),
                Commands.deadline(
                    new WaitCommand(4),
                    Commands.sequence(
                        shooter
                            .setGoal(ShooterConstants.FEED_SPEED)
                            .alongWith(ampMech.runRoller(ShooterConstants.SUCK_IN_TO_AMP_SPEED))
                            .until(ampMech::beamBreakTriggered),
                        stopAllRollers(),
                        shooter
                            .setGoal(ShooterConstants.SUCK_IN_TO_AMP_SPEED)
                            .alongWith(ampMech.runRoller(AmpMechConstants.SUCK_IN_SPEED))
                            .until(ampMech::beamBreakTriggered),
                        new WaitUntilCommand(() -> !ampMech.beamBreakTriggered()),
                        stopAllRollers(),
                        led.strobeLED(Color.kGreenYellow, 0.1))),
                stopAllRollers()));

    // -* RIGHT BUMPER HOLD *- Spinup Shooter to shoot
    operatorController
        .rightBumper()
        .whileTrue(
            shooter
                .setGoal(ShooterConstants.SHOOT_RPM)
                .alongWith(operatorController.rumbleController(GenericHID.RumbleType.kBothRumble, 1)))
        .onFalse(
            shooter
                .stopShooter()
                .alongWith(operatorController.rumbleController(GenericHID.RumbleType.kBothRumble, 0)));

    // -* LEFT BUMPER TAP *- Amp Stow Controller
    operatorController
        .leftBumper()
        .onTrue(ampMech.setPivotGoal(AmpMechConstants.AMP_MECH_IN_ANGLE))
        .onFalse(
            Commands.sequence(
                ampMech.stopRollers(), ampMech.setPivotGoal(AmpMechConstants.AMP_MECH_STOW_ANGLE)));

    // -* RIGHT TRIGGER *- Climber control
    operatorController
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
        .onTrue(climb.runClimb(operatorController::getRightY, operatorController::getLeftY))
        .onFalse(climb.stopClimb());

    // Split setup
    operatorController.povUp().whileTrue(intake.startOutake()).onFalse(intake.retract());
  }

  public Command stopAllRollers() {
    return Commands.sequence(intake.stopRoller(), shooter.stopShooter(), ampMech.stopRollers());
  }

  public Command readyAmpMech() {
    return Commands.sequence(
        intake.ampMechFeed(),
        shooter.setGoal(ShooterConstants.FEED_SPEED),
        ampMech.runRoller(AmpMechConstants.AMP_MECH_ROLLER_SUCK_SPEED));
  }
}
