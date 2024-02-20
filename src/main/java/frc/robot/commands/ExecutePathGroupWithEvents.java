// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;

//follows a path with stop events not to be confused with the other one
public class ExecutePathGroupWithEvents extends SequentialCommandGroup {
  /** Creates a new ExecutePathGroupWithEvents. */
  Command currentCommand;
  List<PathPlannerTrajectory> pathGroup;
  DriveUtil du;
  public ExecutePathGroupWithEvents(DriveUtil du, String filename, HashMap<String, Command> eventMap) {
    this.pathGroup= PathPlanner.loadPathGroup(
      filename,
      Constants.MAX_PATH_VELOCITY, 
      Constants.MAX_PATH_ACCELERATION
    );
    this.du = du;
    for(PathPlannerTrajectory traj : pathGroup){
      addCommands(
        new AutoFollowTrajectorySwerve(
          du,
          traj, 
          new PIDController(Constants.AUTO_X_P, Constants.AUTO_X_I, Constants.AUTO_X_D),
          new PIDController(Constants.AUTO_Y_P, Constants.AUTO_Y_I, Constants.AUTO_Y_D),
          new PIDController(Constants.AUTO_THETA_P, Constants.AUTO_THETA_I, Constants.AUTO_THETA_D)
        )
      );
      for(String name : traj.getEndStopEvent().names){
        Command c=eventMap.get(name);
        if(c != null) addCommands(c);
        else System.err.println("Command "+name+" not defined!");
      }

    }
  }


}
