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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureDefaultCommands();
    drive.configureAutos();
    registerAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
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

    //LED control
    operatorCommandController.a().whileTrue(ledSubsystem.strobeLED(Color.kGreen, 1)).onFalse(ledSubsystem.setColor(Color.kBlack));
    operatorCommandController.b().whileTrue(ledSubsystem.strobeLED(Color.kRed, 1)).onFalse(ledSubsystem.setColor(Color.kBlack));

    driverCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5).and(()-> !intake.hasNote())
      .onTrue(intake.startIntake())
        .onFalse(intake.retract().alongWith(
          ledSubsystem.strobeLED(Color.kWhite, 1).onlyIf(()->intake.hasNote()).withTimeout(1).andThen(ledSubsystem.setColor(Color.kBlack))
        )
      ); 
    
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

    driverCommandController.leftBumper().onTrue(
      ampMech.extend().alongWith(new WaitCommand(100)).until(()->ampMech.atGoal()).andThen(
        ampMech.deposit()
      )
    );

     // Binds the climb to both operator sticks
    operatorCommandController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .5).and(()->!climb.atCurrentLimit()).whileTrue(
      climb.setVoltage(RobotContainer::getOperatorLeftXboxY,RobotContainer::getOperatorRightXboxY) //this is hacky, I don't care.
    ).onFalse(
      climb.StopClimb()
    );

    operatorCommandController.leftBumper().onTrue(ampMech.stopRollers().andThen(ampMech.stow()));

    operatorCommandController.y().onTrue(
      ampMech.prepareGrab()).onFalse(
        readyAmpMech().until(() -> ampMech.beamBreakHit())
        .andThen(
          ampMech.waitUntilBeamBreakIs(false)
          .andThen(
            feedIntoAmpMech().until( () -> ampMech.beamBreakHit() )
            .andThen(
              stopAllRollers().andThen(
                shooter.suckIn().alongWith(ampMech.suckIn()).until(()->ampMech.beamBreakHit()).andThen(
                  ampMech.waitUntilBeamBreakIs(true).andThen(
                    stopAllRollers()
                  )
                )
              )
            )
          )
        ).withTimeout(2).andThen(stopAllRollers())
    );


  }

  public Command readyAmpMech() {
    return intake.ampMechFeed().alongWith(shooter.feed());
  }

  public Command feedIntoAmpMech() {
    return shooter.feedSlow().alongWith( ampMech.suckNote() );
  }

  public Command stopAllRollers() {
    return shooter.stopRollers().alongWith(ampMech.stopRollers()).alongWith(intake.stopRoller());
  }

  public void registerAutoCommands() {
    NamedCommands.registerCommand("Spinup", shooter.startSpinup());
    NamedCommands.registerCommand("Spinup and Shoot", shooter.spinup().withTimeout(1).andThen(intake.feed().withTimeout(.5)).andThen(shooter.stopRollers().alongWith(intake.stopRoller())));
    NamedCommands.registerCommand("Intake 2.5 Seconds", intake.startIntake().alongWith(new WaitCommand(2.5)).andThen(intake.retract()));
    NamedCommands.registerCommand("Intake 1.5 Seconds", intake.startIntake().alongWith(new WaitCommand(2.5)).andThen(intake.retract()));
    NamedCommands.registerCommand("Intake 4 Seconds", intake.startIntake().alongWith(new WaitCommand(2.5)).andThen(intake.retract()));
    NamedCommands.registerCommand("ShooterRoll", shooter.spinup().withTimeout(.5).andThen(intake.feed().withTimeout(.5)).andThen(shooter.stopRollers().alongWith(intake.stopRoller())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  private void configureDefaultCommands() {
    //x and y are swapped because robot's x is forward-backward, while controller x is left-right
    drive.setDefaultCommand(drive.driveRobot(
          RobotContainer::getDriverLeftXboxY,
          RobotContainer::getDriverLeftXboxX,
          RobotContainer::getDriverRightXboxX,
          ()->(RobotContainer.getDriverRightXboxTrigger() > .5)
        )
      );
  }

  public static Command rumbleDriverCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(()->operator.setRumble(rmb, n));
  }

  public static Command rumbleOperatorCommand(GenericHID.RumbleType rmb, double n) {
    return new InstantCommand(()->driver.setRumble(rmb, n));
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
