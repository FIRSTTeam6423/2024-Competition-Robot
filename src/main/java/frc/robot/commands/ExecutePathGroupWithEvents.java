// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drive.DriveConstants;
import frc.robot.Drive.Drive;

//follows a path with stop events not to be confused with the other one
public class ExecutePathGroupWithEvents extends SequentialCommandGroup {
  /** Creates a new ExecutePathGroupWithEvents. */
  Command currentCommand;
  List<PathPlannerPath> pathGroup;
  Drive du;
  public ExecutePathGroupWithEvents(Drive du, String filename, HashMap<String, Command> eventMap) {
    this.pathGroup= PathPlannerAuto.getPathGroupFromAutoFile(filename);
    this.du = du;
    for(PathPlannerPath path : pathGroup){

      addCommands(
        new AutoFollowTrajectorySwerve(
          du,
          path, 
          new PIDController(DriveConstants.AUTO_X_P, DriveConstants.AUTO_X_I, DriveConstants.AUTO_X_D),
          new PIDController(DriveConstants.AUTO_Y_P, DriveConstants.AUTO_Y_I, DriveConstants.AUTO_Y_D),
          new PIDController(DriveConstants.AUTO_THETA_P, DriveConstants.AUTO_THETA_I, DriveConstants.AUTO_THETA_D)
        )
      );
      for(String name : path.getEndStopEvent().names){
        Command c=eventMap.get(name);
        if(c != null) addCommands(c);
        else System.err.println("Command "+name+" not defined!");
      }

    }
  }


}
