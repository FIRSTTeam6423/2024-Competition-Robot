// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CargoUtil extends SubsystemBase {
  /** Creates a new CargoUtil. */
  private CANSparkMax intakepivotmoter;

  private TrapezoidProfile intakeProfile, ampMechProfile;

  public CargoUtil() {
    intakepivotmoter = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);


    
  }

  public void operateCargo(boolean intake){

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
