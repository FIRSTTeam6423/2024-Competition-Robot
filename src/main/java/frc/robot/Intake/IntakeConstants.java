package frc.robot.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public final class IntakeConstants {
    public static final double PIVOT_kG = 0.025; //0.58;//
    public static final double PIVOT_kV = 0;//0.00765858;
    public static final double PIVOT_kS = 0;//0.23125;
    public static final double PIVOT_kA = 0;//0.00086773;

    public static final double PIVOT_P = .01;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;

    public static final int PIVOT_MOTOR = 13; 
    public static final int ROLLER_MOTOR = 14; 
  
    public static final int PIVOT_ENCODER = 5; 

    public static final double PIVOT_OUT_ANGLE = -58;
    public static final double PIVOT_IN_ANGLE = 158;//145;
    public static final double PIVOT_HORIZONTAL_ANGLE = 29; // needs to be confirmed with testing

    public static final double ROLLER_FEED_SHOOTER_SPEED = 1;
    public static final double ROLLER_AMP_MECH_FEED_SPEED = .2; //.1
    public static final double ROLLER_INTAKE_SPEED = -.75;
    public static final double ROLLER_OUTAKE_SPEED = .75;

    public static final double PIVOT_ENCODER_OFFSET_DEGREES = 43;
    public static final double PIVOT_DEADBAND_DEGREES = 2.17;

    public static final double MAX_VELOCITY_DEG_PER_SEC = 1250;
    public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 1100;//700;//500;

    public static final double ROLLER_NOTEFIX_TIMEOUT = 1; //unused im pretty sure
}
