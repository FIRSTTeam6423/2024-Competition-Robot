// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AmpMech.AmpMech;
import frc.robot.Climb.Climb;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.OperateDrive;
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
  private static final Drive drive = new Drive();
  private static final Climb climb = new Climb();

  private static XboxController driver = new XboxController(0);
  private static XboxController operator = new XboxController(1);
  private static CommandXboxController operatorCommandController = new CommandXboxController(1);
  private static CommandXboxController driverCommandController = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser;

  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private AmpMech ampMech = new AmpMech();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();

  private GenericEntry intakeVoltEntry = Shuffleboard.getTab("Intake").add("Pivot Volts", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  private PIDController lockRotationController = new PIDController(.015, 0, 0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    lockRotationController.enableContinuousInput(-180, 180);
    configureDefaultCommands();
    configureBindings();
    configureDefaultCommands();
    drive.configureAutos();
    registerAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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

    // LED control
    operatorCommandController.a().whileTrue(ledSubsystem.strobeLED(Color.kGreen, .25))
        .onFalse(ledSubsystem.setColor(Color.kBlack));
    operatorCommandController.b().whileTrue(ledSubsystem.strobeLED(Color.kRed, .25))
        .onFalse(ledSubsystem.setColor(Color.kBlack));

    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
        .onTrue(intake.startIntake())
        .onFalse(intake.retract());

    Trigger hasNoteTrigger = new Trigger(intake::hasNote);
    hasNoteTrigger.onTrue(
        ledSubsystem.strobeLED(Color.kWhite, .1).onlyIf(() -> intake.hasNote()).withTimeout(1.5)
            .andThen(ledSubsystem.setColor(Color.kBlack)));

    driverCommandController.rightBumper().onTrue(intake.shooterFeed()).onFalse(intake.stopRoller());

    // Shooter flywheels SYSID control
    // driverCommandController.y().whileTrue(shooter.runQuasistatic(SysIdRoutine.Direction.kForward));
    // driverCommandController.b().whileTrue(shooter.runQuasistatic(SysIdRoutine.Direction.kReverse));

    // driverCommandController.x().whileTrue(shooter.runDynamic(SysIdRoutine.Direction.kForward));
    // driverCommandController.a().whileTrue(shooter.runDynamic(SysIdRoutine.Direction.kReverse));

    // if operator doesn't do spinup, shoot button will spinup anyway
    // if operator doesn't prime for amp deposit, amp release button on driver will
    // NOT prime. WILL DO NOTHING
    operatorCommandController.rightBumper().whileTrue(
        shooter.spinup().alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 1))
            .until(() -> driver.getRightBumper()).andThen(
                intake.shooterFeed().withTimeout(1).andThen(
                    intake.stopRoller().asProxy().withInterruptBehavior(InterruptionBehavior.kCancelIncoming))))
        .onFalse(shooter.stopRollers().alongWith(rumbleOperatorCommand(GenericHID.RumbleType.kBothRumble, 0)));

    // Strobes blue LEDs when shooter is at RPM
    Trigger atRPMTrigger = new Trigger(shooter::atRPM);
    atRPMTrigger.onTrue(ledSubsystem.strobeLED(Color.kBlue, 0.05)).onFalse(ledSubsystem.setColor(Color.kBlack));

    driverCommandController.leftBumper().onTrue(
        ampMech.extend().alongWith(new WaitCommand(100)).until(() -> ampMech.atGoal()).andThen(
          ampMech.deposit()
        )
      );

    // Binds the climb to both operator sticks
    operatorCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5)
        .and(() -> !climb.atCurrentLimit()).whileTrue(
            climb.setVoltage(RobotContainer::getOperatorRightXboxY, RobotContainer::getOperatorLeftXboxY) // this is
                                                                                                          // hacky, I
                                                                                                          // don't care.
        ).onFalse(
            climb.StopClimb());

    operatorCommandController.leftBumper().onTrue(ampMech.prepareGrab()).onFalse(ampMech.stopRollers().andThen(ampMech.stow()));

    operatorCommandController.x().and(()->!intake.fullyHasNote()).onTrue(
        ampMech.suckBack().alongWith(
            shooter.suckBack()).alongWith(
                intake.suckBack()))
        .onFalse(stopAllRollers().andThen(ampMech.stow()));
 
    operatorCommandController.y().onTrue(
      ampMech.prepareGrab()
    ).onFalse(
        new WaitUntilCommand(() -> intake.atGoal()).andThen(
          readyAmpMech().until(() -> ampMech.beamBreakHit())
            .andThen(new WaitUntilCommand(() -> !ampMech.beamBreakHit()))
              .andThen(
                feedIntoAmpMech().until(() -> ampMech.beamBreakHit())
                  .andThen(
                    stopAllRollers().andThen(
                      shooter.suckIn().alongWith(ampMech.suckIn()).until(() -> ampMech.beamBreakHit())
                      .andThen(
                        ampMech.waitUntilBeamBreakIs(false).andThen(
                          stopAllRollers().andThen(
                            ledSubsystem.strobeLED(Color.kGreenYellow, 0.1)
                          )
                        )
                      )
                    )
                  )
                ).withTimeout(4).andThen(stopAllRollers())
        )
      );

    operatorCommandController.povUp().whileTrue(
        intake.startOutake()).onFalse(intake.retract());

    operatorCommandController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .5).onTrue(ampMech.switchCode());
  }
  
  public Command readyAmpMech() {
    return intake.ampMechFeed().alongWith(shooter.feed()).alongWith(ampMech.suckNote());
  }

  public Command feedIntoAmpMech() {
    return shooter.feedSlow().alongWith(ampMech.suckNote());
  }

  public Command stopAllRollers() {
    return shooter.stopRollers().alongWith(ampMech.stopRollers()).alongWith(intake.stopRoller());
  }

  public void registerAutoCommands() {

    NamedCommands.registerCommand("Intake Until Note", intake.startIntake().andThen(intake.retract()));

    NamedCommands.registerCommand("Retract and Shoot",
        intake.retract().andThen(new WaitUntilCommand(()->intake.atGoal())).andThen(
          spinupShooterAndShootAtRPM()
        )
    );
    
  }

  public Command spinupShooterAndShootAtRPM() {
    return shooter.spinup().until(()->shooter.atRPM()).andThen(
      intake.shooterFeed().withTimeout(.25).andThen(intake.stopRoller())
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);

  private void configureDefaultCommands() {
    // x and y are swapped becausrobot's x is forward-backward, while controller x
    // is left-right
    drive.setDefaultCommand(drive.driveRobot(
        RobotContainer::getDriverLeftXboxY,
        RobotContainer::getDriverLeftXboxX,
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
          return RobotContainer.getDriverRightXboxX();
        },
        () -> (RobotContainer.getDriverLeftXboxTrigger() > .5)));
    enabledTrigger.whileTrue(ledSubsystem.enabledIdle());
    enabledTrigger.whileFalse(ledSubsystem.disabledIdle());

    ledSubsystem.setDefaultCommand(ledSubsystem.disabledIdle().onlyWhile(DriverStation::isDisabled)
         .andThen(ledSubsystem.enabledIdle().onlyWhile(DriverStation::isEnabled)));
     ledSubsystem.disabledIdle().schedule();
    shooter.setDefaultCommand(shooter.spinup().onlyWhile(DriverStation::isAutonomous)
        .andThen(shooter.stopRollers().withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
    // intake.setDefaultCommand(intake.setPivotVolts(()->{return
    // intakeVoltEntry.getDouble(0);}));
    driverCommandController.povDown().onTrue(Commands.runOnce(() -> {
      drive.flipOrientation();
    }));

  }

  public static Command rumbleDriverCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(() -> operator.setRumble(rmb, n));
  }

  public static Command rumbleOperatorCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(() -> driver.setRumble(rmb, n));
  }

  public static double getOperatorRightXboxY() {
    return operator.getRightY();
  }

  public static double getOperatorLeftXboxY() {
    return operator.getLeftY();
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
