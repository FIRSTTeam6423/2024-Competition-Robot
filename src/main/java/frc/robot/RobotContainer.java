
package frc.robot;

// * SUBSYSTEMS
import frc.robot.AmpMech.AmpMech;
import frc.robot.Climb.Climb;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;
import frc.robot.Drive.Drive;

// * CONSTANTS
import frc.robot.AmpMech.AmpMechConstants;
import frc.robot.Climb.ClimbConstants;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Shooter.ShooterConstants;
import frc.robot.Drive.DriveConstants;

// * COMMANDS
import frc.robot.commands.OperateDrive;
import frc.robot.commands.Autos;

// -----------------------------------------------------------------

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.controller.PIDController;

// -----------------------------------------------------------------

public class RobotContainer {
  // * ------ SUBSYSTEMS ------
  private final Drive drive = new Drive();
  private final Climb climb = new Climb();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final AmpMech ampMech = new AmpMech();
  private final LEDSubsystem led = new LEDSubsystem();

  // * ------ AUTO (womp womp) ------
  public final SendableChooser<Command> autoSelector = Autos.configureAutos(drive, intake, climb, ampMech, shooter);

  // * ------ CONTROLLERS ------
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);
  public static CommandXboxController driverCommandController = new CommandXboxController(0);
  public static CommandXboxController operatorCommandController = new CommandXboxController(1);

  // TEMP

  private PIDController lockRotationController = new PIDController(.021, 0, .001);
  public static boolean allowDeposit;

  private Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
  private Trigger autonomousTriggerr = new Trigger(DriverStation::isAutonomousEnabled);
  private Trigger teleopTrigger = new Trigger(DriverStation::isTeleopEnabled);

  // Contains subsystems 
  public RobotContainer() {
    lockRotationController.enableContinuousInput(-180, 180);
    configureDefaultCommands();
    configureBindings();
    configureDefaultCommands(); // ! why dis called twice :skull:
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
    new Trigger(intake::hasNote).onTrue(
      led.strobeLED(Color.kWhite, .1)
      .onlyIf(() -> intake.hasNote()) // ! Is this even neccesary?
      .withTimeout(1.5)
      .andThen(led.setColor(Color.kBlack))
    );

    // Blue LED strobe when shooter reaches RPM
    new Trigger(shooter::atRPM).onTrue(
      led.strobeLED(Color.kBlue, 0.05)
    ).onFalse(
      led.setColor(Color.kBlack)
    );

    // * ------ CONTROLLER BINDS ------
    
    // ---- DRIVER ----
    
    // Feed note to shooter
    driverCommandController.rightBumper().onTrue(
      intake.shooterFeed()
    ).onFalse(
      new WaitCommand(.75)
      .onlyIf(shooter::atRPM)
      .andThen(
        intake.stopRoller()
      )
    );

    // Amp Control 
    driverCommandController.leftBumper().onTrue(
      ampMech.extend()
      .alongWith(
        new WaitCommand(100)
      )
      .until(() -> ampMech.atGoal())
      .andThen(
        ampMech.deposit()
      )
    );

    // Intake control
    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5).onTrue(
      intake.startIntake()
    ).onFalse(
      intake.retract()
    );

    // ---- OPERATOR ----
    
    // LED green signal
    operatorCommandController.a().whileTrue(
      led.strobeLED(Color.kGreen, .25)
    ).onFalse(
      led.setColor(Color.kBlack)
    );

    // LED red signal
    operatorCommandController.b().whileTrue(
      led.strobeLED(Color.kRed, .25)
    ).onFalse(
      led.setColor(Color.kBlack)
    );

    // Unload amp (Why is this named suck back lmfao)
    operatorCommandController.x().and(() -> !intake.fullyHasNote()).onTrue(
      ampMech.suckBack().alongWith(
        intake.suckBack()
      ).unless(shooter::rightTriggerPressed)
    ).onFalse(
      intake.stopRoller()
      .alongWith(
        ampMech.stopRollers()
      )
    );

    // Amp handoff control 
    // ! TODO Rewrite this :squak:
    operatorCommandController.y().onTrue(
      ampMech.prepareGrab().andThen(ampMech.allowDepostFalse())
    );

    // Spinup Shooter to shoot
    operatorCommandController.rightBumper().whileTrue(
      shooter.spinup()
      .alongWith(
        rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 1)
      )
    ).onFalse(
      shooter.stopRollers()
      .alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 0))
    );

    // Amp Stow Controller 
    operatorCommandController.leftBumper().onTrue(
      ampMech.prepareGrab()    
    ).onFalse(
      ampMech.stopRollers()
      .andThen(
        ampMech.stow()
      )
    );

    // Switch to amp test code 
    operatorCommandController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .5).onTrue(
      ampMech.switchCode()
    );

    // Climber control
    operatorCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
      .and(() -> !climb.atCurrentLimit()).whileTrue(
        climb.setVoltage(operator::getRightY, operator::getLeftY)
      ).onFalse(
        climb.StopClimb()
      );

    // Flip the flipping drive
    operatorCommandController.povRight().onTrue(Commands.runOnce(()->drive.manually_invert_drive()));

    // Split setup
    operatorCommandController.povUp().whileTrue(intake.startOutake()).onFalse(intake.retract());
    
  }
  
  // TODO clean up everything under here

  private void configureDefaultCommands() {
    // x and y are swapped becausrobot's x is forward-backward, while controller x
    // is left-right
    drive.setDefaultCommand(
      drive.driveRobot(
        driver::getLeftY,
        driver::getLeftX,
        () -> {
          if (driver.getYButton()) {
            return -lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), 0); // BASE ON ALLIANCE
                                                                                                     // COLOR MUST BE
                                                                                                     // MIRRORED
          }
          if (driver.getXButton()) {
            return -lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), 45);
          }
          if (driver.getBButton()) {
            return -lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), -45);
          }
          if (driver.getAButton()){
            return -lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), 90);
          }
          return driver.getRightX();
        },
        () -> (driver.getLeftTriggerAxis() > .5)
      )
    );
   enabledTrigger.whileTrue(led.enabledIdle());
  enabledTrigger.whileFalse(led.disabledIdle());

   led.setDefaultCommand(led.disabledIdle().onlyWhile(DriverStation::isDisabled)
          .andThen(led.enabledIdle().onlyWhile(DriverStation::isEnabled)));
    led.disabledIdle().schedule();
   //autonomousTrigger.whileTrue(Commands.runOnce(()->{System.out.println("hi");}));
   //teleopTrigger.onTrue(stopAllRollers());
   shooter.setDefaultCommand(shooter.spinup().onlyWhile(DriverStation::isAutonomous)
        .andThen(shooter.stopRollers().withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
    // intake.setDefaultCommand(intake.setPivotVolts(()->{return
    // intakeVoltEntry.getDouble(0);}));
    driverCommandController.povDown().onTrue(Commands.runOnce(() -> {
      drive.flipOrientation();
    }));
  }

} 
