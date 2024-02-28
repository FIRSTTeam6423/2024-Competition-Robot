package frc.robot.Intake;

public final class IntakeConstants {
    public static final double INTAKE_PIVOT_kG = 0; //0.58;//
    public static final double INTAKE_PIVOT_kV = 0;//0.00765858;
    public static final double INTAKE_PIVOT_kS = 0;//0.23125;
    public static final double INTAKE_PIVOT_kA = 0;//0.00086773;

    public static final double INTAKE_PIVOT_P = .015;
    public static final double INTAKE_PIVOT_I = 0;
    public static final double INTAKE_PIVOT_D = 0;

    public static final int INTAKE_PIVOT_MOTOR = 13; 
    public static final int INTAKE_ROLLER_MOTOR = 14; 
  
    public static final int INTAKE_PIVOT_ENCODER = 5; 

    public static final double INTAKE_PIVOT_OUT_ANGLE = -59;
    public static final double INTAKE_PIVOT_IN_ANGLE = 158;//145;

    public static final double INTAKE_ROLLER_FEED_SPEED = 1;
    public static final double INTAKE_ROLLER_AMP_MECH_FEED_SPEED = .25;
    public static final double INTAKE_ROLLER_INTAKE_SPEED = -.75;

    public static final double INTAKE_PIVOT_ENCODER_OFFSET_DEGREES = 43;
    public static final double INTAKE_PIVOT_DEADBAND_DEGREES = 2.17;

    public static final double INTAKE_MAX_VELOCITY_DEG_PER_SEC = 450;
    public static final double INTAKE_MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 700;//500;

    public static final double ROLLER_NOTEFIX_TIMEOUT = 1;
}
