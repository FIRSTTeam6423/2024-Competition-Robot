package frc.robot.Shooter;

public class ShooterConstants {
    
    public static final int LEFT_MOTOR = 11;
    public static final int RIGHT_MOTOR = 12;
    //4.6874E-05
    public static final double LEFT_ROLLER_P = 2.15 * Math.pow(10, -1); //-5., 4.6874
    public static final double RIGHT_ROLLER_P = 2.15 * Math.pow(10, -1); //-5
    public static final double LEFT_ROLLER_I = 7;
    public static final double RIGHT_ROLLER_I = 0;
    public static final double LEFT_ROLLER_D = 0;
    public static final double RIGHT_ROLLER_D = 0;

    public static final double SHOOT_RPM = 5000;
    public static final double AMP_MECH_FEED_SPEED = 900; //initial feed speed into amp mech
    public static final double AMP_MECH_SUCK_IN_SPEED = -750; //note fix speed
    public static final double kS = 0.3742;
    public static final double kV = 1.255 * Math.pow(10, -1);//3.6 * Math.pow(10, -2);
    public static final double kA = 4.5978 * Math.pow(10, -4);
    public static final double AMP_MECH_SUCK_BACK_SPEED = -1200;
}
