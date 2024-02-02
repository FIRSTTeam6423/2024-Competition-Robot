// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int AMP_MECH_PIVOT_MOTOR = 9;
  public static final int AMP_MECH_ROLLER_MOTOR = 10;

  public static final int LEFT_SHOOTER_MOTOR = 11;
  public static final int RIGHT_SHOOTER_MOTOR = 12;
  
  public static final int INTAKE_PIVOT_MOTOR = 13; 
  public static final int INTAKE_ROLLER_MOTOR_1 = 14; 
  public static final int INTAKE_ROLLER_MOTOR_2 = 15; 

  public static final int INTAKE_PIVOT_ENCODER = 0; //TBDdddd
  public static final int AMP_MECH_PIVOT_ENCODER = 0; //TBD

  public static final double INTAKE_PIVOT_P = 0;
  public static final double INTAKE_PIVOT_I = 0;
  public static final double INTAKE_PIVOT_D = 0;
  
  public static final double AMP_MECH_PIVOT_P = 0;
  public static final double AMP_MECH_PIVOT_I = 0;
  public static final double AMP_MECH_PIVOT_D = 0;
  
  public static final double INTAKE_PIVOT_kG = 1.05; //0.58;//
  public static final double INTAKE_PIVOT_kV = 0;//0.00765858;
  public static final double INTAKE_PIVOT_kS = 0;//0.23125;
  public static final double INTAKE_PIVOT_kA = 0;//0.00086773;
   
  public static final double AMP_MECH_PIVOT_kG = 0.5;//1.3802;
  public static final double AMP_MECH_PIVOT_kV = 0;//0.016472;
  public static final double AMP_MECH_PIVOT_kS = 0;//-0.6777;
  public static final double AMP_MECH_PIVOT_kA = 0;//0.0098395;

  public static final double INTAKE_PIVOT_OUT_ANGLE = 45;
  public static final double INTAKE_PIVOT_IN_ANGLE = 0;
  public static final int SHOOTER_ROLLER_MOTOR1 = 0;
  public static final int SHOOTER_ROLLER_MOTOR2 = 0;
  public static final double SHOOTER_ROLLER_P = 0;
  public static final double SHOOTER_ROLLER_I = 0;
  public static final double SHOOTER_ROLLER_D = 0;
  public static final double INTAKE_ROLLER_REVERSE_SPEED = 0;
  public static final double INTAKE_ROLLER_SPEED = 0;
  public static final double SHOOTER_ROLLER_SPINUP_SPEED = 0;
  public static final double AMP_MECH_PIVOT_HANDOFF_ANGLE = 0;
  public static final double SHOOTER_ROLLER_HANDOFF_SPEED = 0;
  public static final double AMP_MECH_ROLLER = 0;
  
  public static final double AMP_MECH_PIVOT_ENCODER_OFFSET = 0;
  public static final double INTAKE_PIVOT_ENCODER_OFFSET = 0;
public static final double SHOOTING_TIME = 0;
public static final double AMP_MECH_PIVOT_DEPOSIT_ANGLE = 0;
public static final double AMP_MECH_PIVOT_IDLE_ANGLE = 0;
public static final double SHOOTER_ROLLER_MIN_VELOCITY = 0;
public static final double SHOOTER_ROLLER_MAX_VELOCITY = 0;
public static final double SHOOTER_ROLLER_DEADBAND_VELOCITY = 0;
public static final double AMP_MECH_DEPOSIT_SPEED = 0;


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
