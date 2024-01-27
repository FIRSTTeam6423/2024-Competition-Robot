// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int AMP_PIVOT_MOTOR = 9;
  public static final int AMP_ROLLER_MOTOR = 10;

  public static final int LEFT_SHOOTER_MOTOR = 11;
  public static final int RIGHT_SHOOTER_MOTOR = 12;
  
  public static final int INTAKE_PIVOT_MOTOR = 13; 
  public static final int INTAKE_ROLLER_MOTOR = 14; 


  public static final int INTAKE_LIMIT_SWITCH = 8;
  public static final int ARM_LIMIT_SWITCH = 5;
  public static final int GRABBER_LIMIT_SWITCH_ID = 6;


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
