package frc.robot;

import static frc.robot.subsystems.AmpMech.AmpMechConstants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbCommands;
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

  // * ------ COMMANDS ------
  public final ClimbCommands climbCommands;

  // * ------ CONTROLLERS ------
  public static CommandXboxController driverController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);
  public static XboxController driver = driverController.getHID();
  public static XboxController operator = operatorController.getHID();
  public static XboxControllerSim driverSim = new XboxControllerSim(driver);

  // Contains subsystems
  public RobotContainer() {

    // Initalizes IO hardware for subsystems
    if (Robot.isReal()) {
      drive = new Drive();
      climb = new Climb(new ClimbIOReal());
      intake = Intake.getInstance();
      shooter = new Shooter(new ShooterIOReal());
      ampMech = new AmpMech(new AmpMechIOReal());
      led = new LEDSubsystem();
    } else {
      drive = new Drive();
      climb = new Climb(new ClimbIOSim());
      intake = Intake.getInstance();
      shooter = new Shooter(new ShooterIOSim());
      ampMech = new AmpMech(new AmpMechIOSim());
      led = new LEDSubsystem();
    }
    autoSelector = Autos.configureAutos(drive, intake, climb, ampMech, shooter);
    climbCommands = new ClimbCommands(climb);

    drive.lockRotationController.enableContinuousInput(-180, 180);
    configureBindings();
    configureDefaultCommands();
    SmartDashboard.putData("Auto Chooser", autoSelector);
    intake.retract().schedule();
  }

  // ------ RUMBLE COMMANDS ------

  public static Command rumbleDriverCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(() -> operator.setRumble(rmb, n));
  }

  public static Command rumbleOperatorCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(() -> driver.setRumble(rmb, n));
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
                ampMech.setPivotGoal(AMP_MECH_OUT_ANGLE),
                new WaitUntilCommand(ampMech::atGoal),
                ampMech.runRoller(AMP_MECH_DEPOSIT_SPEED)));

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
                .runRoller(SUCK_BACK_SPEED)
                .alongWith(intake.unload())
                .unless(() -> operator.getRightBumper()))
        .onFalse(intake.stopRoller().alongWith(ampMech.stopRollers()));

    // -* Y BUTTON TAP *- Amp handoff control
    operatorController
        .y()
        .onTrue(ampMech.setPivotGoal(AMP_MECH_IN_ANGLE))
        .onFalse(
            Commands.sequence(
                new WaitUntilCommand(intake::atGoal),
                readyAmpMech().until(ampMech::beamBreakTriggered),
                new WaitUntilCommand(() -> !ampMech.beamBreakTriggered()),
                Commands.deadline(
                    new WaitCommand(4),
                    Commands.sequence(
                        shooter
                            .setGoal(AMP_MECH_FEED_SPEED)
                            .alongWith(ampMech.runRoller(AMP_MECH_ROLLER_SUCK_SPEED))
                            .until(ampMech::beamBreakTriggered),
                        stopAllRollers(),
                        shooter
                            .setGoal(AMP_MECH_SUCK_IN_SPEED)
                            .alongWith(ampMech.runRoller(SUCK_IN_SPEED))
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
                .setGoal(SHOOT_RPM)
                .alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 1)))
        .onFalse(
            shooter
                .stopShooter()
                .alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 0)));

    // -* LEFT BUMPER TAP *- Amp Stow Controller
    operatorController
        .leftBumper()
        .onTrue(ampMech.setPivotGoal(AMP_MECH_IN_ANGLE))
        .onFalse(
            Commands.sequence(ampMech.stopRollers(), ampMech.setPivotGoal(AMP_MECH_STOW_ANGLE)));

    // -* LEFT TRIGGER *- Switch to amp test code
    /*
     * operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value,
     * .5).onTrue( ampMech.switchCode() );
     */

    // -* RIGHT TRIGGER *- Climber control
    operatorController
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
        .onTrue(climbCommands.runClimb(operator::getRightY, operator::getLeftY))
        .onFalse(climbCommands.stopClimb());

    // Flip the flipping drive
    operatorController.povRight().onTrue(Commands.runOnce(() -> drive.manually_invert_drive()));

    // Split setup
    operatorController.povUp().whileTrue(intake.startOutake()).onFalse(intake.retract());
  }

  public Command stopAllRollers() {
    return Commands.sequence(intake.stopRoller(), shooter.stopShooter(), ampMech.stopRollers());
  }

  public Command readyAmpMech() {
    return Commands.sequence(
        intake.ampMechFeed(),
        shooter.setGoal(AMP_MECH_FEED_SPEED),
        ampMech.runRoller(AMP_MECH_ROLLER_SUCK_SPEED));
  }

  // ------ DEFAULT SUBSYSTEM STATES ------
  private void configureDefaultCommands() {
    // x and y are swapped becausrobot's x is forward-backward, while controller x
    // is left-right
    drive.setDefaultCommand(
        drive.driveRobot(
            driver::getLeftY,
            driver::getLeftX,
            () -> {
              if (driver.getYButton()) {
                return -drive.lockRotationController.calculate(
                    drive.getPose().getRotation().getDegrees(), 0);
              }
              if (driver.getXButton()) {
                return -drive.lockRotationController.calculate(
                    drive.getPose().getRotation().getDegrees(), 45);
              }
              if (driver.getBButton()) {
                return -drive.lockRotationController.calculate(
                    drive.getPose().getRotation().getDegrees(), -45);
              }
              if (driver.getAButton()) {
                return -drive.lockRotationController.calculate(
                    drive.getPose().getRotation().getDegrees(), 90);
              }
              return driver.getRightX();
            },
            () -> (driver.getLeftTriggerAxis() > .5)));
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
            .setGoal(SHOOT_RPM)
            .onlyWhile(DriverStation::isAutonomous)
            .andThen(
                shooter.stopShooter().withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.flipOrientation();
                }));
  }
}
