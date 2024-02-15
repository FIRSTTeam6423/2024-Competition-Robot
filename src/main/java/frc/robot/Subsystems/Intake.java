// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private DigitalInput[] intakeLimitSwitches = {
    new DigitalInput(7), // TODO gotta fix outputs
    new DigitalInput(8),
    new DigitalInput(9),
  };
  
  /** Creates a new Intake. */
  public Intake() {

  }

  public boolean hasNote() {
    for(DigitalInput intakeSwitch: intakeLimitSwitches){
      if (!intakeSwitch.get()){
        return true;
      }
    }
    return false;
  }

  public Command extend() {
    //return run(()->)dw
  }

  public Command retract() {
    
  }
}
