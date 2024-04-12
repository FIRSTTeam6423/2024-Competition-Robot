
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
import frc.robot.commands.ClimbCommands;

// -----------------------------------------------------------------

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

// -----------------------------------------------------------------

public class RobotContainer {
  // * ------ SUBSYSTEMS ------
  private final Drive drive = new Drive();
  private final Climb climb = Climb.getInstance();
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

  // Contains subsystems 
  public RobotContainer() {
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
    new Trigger(intake::hasNote).onTrue(
      led.strobeLED( Color.kWhite, .1 )
      .onlyIf( () -> intake.hasNote() ) // ! Is this even neccesary?
      .withTimeout(1.5)
      .andThen(led.setColor(Color.kBlack))
    );

    // Blue LED strobe when shooter reaches RPM
    new Trigger(shooter::atRPM).onTrue(
      led.strobeLED(Color.kBlue, 0.05)
    ).onFalse(
      led.setColor(Color.kBlack)
    );

    // ---- DRIVER BINDS ----
    
    // -* RIGHT BUMPER TAP *- Feed note to shooter
    driverCommandController.rightBumper().onTrue(
      intake.shooterFeed()
    ).onFalse(
      new WaitCommand(.75)
      .onlyIf(shooter::atRPM)
      .andThen(
        intake.stopRoller()
      )
    );

    // -* LEFT BUMPER TAP *- Amp Control 
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

    // -* RIGHT TRIGGER *- Intake control
    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5).onTrue(
      intake.startIntake()
    ).onFalse(
      intake.retract()
    );

    // ---- OPERATOR BINDS ----
    
    // -* A BUTTON TAP *- LED green signal
    operatorCommandController.a().whileTrue(
      led.strobeLED(Color.kGreen, .25)
    ).onFalse(
      led.setColor(Color.kBlack)
    );

    // -* B BUTTON TAP *- LED red signal
    operatorCommandController.b().whileTrue(
      led.strobeLED(Color.kRed, .25)
    ).onFalse(
      led.setColor(Color.kBlack)
    );

    // -* X BUTTON TAP *- Unload amp (Why is this named suck back lmfao)
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

    // -* Y BUTTON TAP *- Amp handoff control 
    // This is fucking cancer
    operatorCommandController.y().onTrue(
      ampMech.prepareGrab()
      .andThen(
        ampMech.allowDepostFalse()
      )
    ).onFalse(
      new WaitUntilCommand(() -> intake.atGoal())
      .andThen(
        intake.ampMechFeed()
        .alongWith(
          shooter.feed()
        ).alongWith(
          ampMech.suckNote()
        ).until(() -> ampMech.beamBreakHit())
        .andThen(
          new WaitUntilCommand(() -> !ampMech.beamBreakHit())
        )
        .andThen(
          shooter.feedSlow()
          .alongWith(
            ampMech.suckNote()
          ).until(() -> ampMech.beamBreakHit())
          .andThen(
            shooter.stopRollers()
            .alongWith(
              ampMech.stopRollers()
            ).alongWith(
              intake.stopRoller()
            ).andThen(
              shooter.suckIn()
              .alongWith(
                ampMech.suckIn()
              ).until(() -> ampMech.beamBreakHit())
              .andThen(
                ampMech.waitUntilBeamBreakIs(false)
                .andThen(
                  shooter.stopRollers()
                  .alongWith(
                    ampMech.stopRollers()
                  ).alongWith(
                    intake.stopRoller()
                  ).andThen(
                    led.strobeLED(Color.kGreenYellow, 0.1)
                    )
                  )
                )
              )
            )
          ).withTimeout(4).andThen(shooter.stopRollers().alongWith(ampMech.stopRollers()).alongWith(intake.stopRoller()))
      )
    );

    // -* RIGHT BUMPER HOLD *- Spinup Shooter to shoot
    operatorCommandController.rightBumper().whileTrue(
      shooter.spinup()
      .alongWith(
        rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 1)
      )
    ).onFalse(
      shooter.stopRollers()
      .alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 0))
    );

    // -* LEFT BUMPER TAP *- Amp Stow Controller 
    operatorCommandController.leftBumper().onTrue(
      ampMech.prepareGrab()    
    ).onFalse(
      ampMech.stopRollers()
      .andThen(
        ampMech.stow()
      )
    );

    // -* LEFT TRIGGER *- Switch to amp test code 
    operatorCommandController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .5).onTrue(
      ampMech.switchCode()
    );

    // -* RIGHT TRIGGER *- Climber control
    operatorCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
    .and(() -> !climb.atCurrentLimit()).whileTrue(
      climb.setVoltage(operator::getRightY, operator::getLeftY)
    ).onFalse(
      climb.StopClimb()
    );
    // ! This might be reall stupid lmfao EDIT: THIS WAS REALLY STUPID LMFAO
    /* operatorCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
    .and( () -> !climb.atCurrentLimit() ).whileTrue(
      ClimbCommands.climbChain(operator, climb)
    ).onFalse(
      ClimbCommands.climbStop(climb)
    ); */

    // Flip the flipping drive
    operatorCommandController.povRight().onTrue(Commands.runOnce(()->drive.manually_invert_drive()));

    // Split setup
    operatorCommandController.povUp().whileTrue(intake.startOutake()).onFalse(intake.retract());
    
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
            return -drive.lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), 0);
          }
          if (driver.getXButton()) {
            return -drive.lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), 45);
          }
          if (driver.getBButton()) {
            return -drive.lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), -45);
          }
          if (driver.getAButton()){
            return -drive.lockRotationController.calculate(drive.getPose().getRotation().getDegrees(), 90);
          }
          return driver.getRightX();
        },
        () -> (driver.getLeftTriggerAxis() > .5)
      )
    );
    new Trigger(DriverStation::isDisabled).whileTrue(led.enabledIdle());
    new Trigger(DriverStation::isEnabled).whileFalse(led.disabledIdle());
   
    // LED disabled idle mode
    led.setDefaultCommand(
      led.disabledIdle().onlyWhile( DriverStation::isDisabled )
      .andThen(
        led.enabledIdle().onlyWhile( DriverStation::isEnabled )
      )
    );
    led.disabledIdle().schedule();

    // Shooter Auto mode
    shooter.setDefaultCommand(
      shooter.spinup().onlyWhile( DriverStation::isAutonomous )
      .andThen(
        shooter.stopRollers()
          .withInterruptBehavior( InterruptionBehavior.kCancelSelf )
      )
    );

    driverCommandController.povDown().onTrue(Commands.runOnce(() -> { drive.flipOrientation(); }));
  }
} 
