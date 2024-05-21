package frc.robot.commands;

import static frc.robot.Constants.DriveConstants;

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
import frc.robot.subsystems.AmpMech.AmpMech;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class Autos {
  public static SendableChooser<Command> configureAutos(
      Drive drive, Intake intake, Climb climb, AmpMech ampMech, Shooter shooter) {
    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::resetOdometry,
        drive::getChassisSpeeds,
        drive::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(DriveConstants.AUTO_X_P, 0.0, 0.05),
            new PIDConstants(DriveConstants.AUTO_THETA_P, 0.0, 0.05),
            4.5,
            Units.inchesToMeters(16.6),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drive);

    NamedCommands.registerCommand(
        "Intake Until Note", intake.startIntake().andThen(intake.retract()));

    NamedCommands.registerCommand(
        "Retract and Shoot",
        intake
            .retract()
            .andThen(
                new WaitUntilCommand(() -> intake.atGoal())
                    .andThen(intake.shooterFeed().withTimeout(.25).andThen(intake.stopRoller()))));

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    return chooser;
  }
}
