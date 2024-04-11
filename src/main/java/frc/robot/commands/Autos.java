package frc.robot.commands;

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


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Autos {

  public static SendableChooser<Command> configureAutos(Drive drive, Intake intake, Climb climb, AmpMech ampMech, Shooter shooter) {
    AutoBuilder.configureHolonomic(
      drive::getPose,
      drive::resetPose,
      drive::getRobotRelativeSpeeds,
      drive::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(DriveConstants.AUTO_X_P, 0.0, 0.0),
        new PIDConstants(DriveConstants.AUTO_THETA_P, 0.0, 0.0),
        4.5,
        Units.inchesToMeters(16.6),
        new ReplanningConfig()
      ), () -> { var alliance = DriverStation.getAlliance(); if (alliance.isPresent()) { return alliance.get() == DriverStation.Alliance.Red; } return false; },
      drive
    );

    NamedCommands.registerCommand("Intake Until Note", 
      intake.startIntake()
      .andThen(intake.retract())
    );
    
    NamedCommands.registerCommand("Retract and Shoot",
      intake.retract()
      .andThen(
        new WaitUntilCommand(() -> intake.atGoal())
        .andThen(
          intake.shooterFeed()
          .withTimeout(.25)
          .andThen(intake.stopRoller())
        )
      )
    );

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    return chooser;
  }
}
