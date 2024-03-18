package frc.robot.AmpMech;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycle;

public final class AmpMechConstants {

    public static final int AMP_MECH_PIVOT_ENCODER = 4;
    public static final int AMP_MECH_PIVOT_MOTOR = 9;
    public static final int AMP_MECH_ROLLER_MOTOR = 10;

    public static final double AMP_MECH_PIVOT_kS = 0;
    public static final double AMP_MECH_PIVOT_kG = 0;
    public static final double AMP_MECH_PIVOT_kV = 0;
    public static final double AMP_MECH_PIVOT_kA = 0;

    public static final double AMP_MECH_PIVOT_P = 0.02; //0.025
    public static final double AMP_MECH_PIVOT_I = 0;
    public static final double AMP_MECH_PIVOT_D = 0;

    public static final double AMP_MECH_MAX_VELOCITY_DEG_PER_SEC = 5000;
    public static final double AMP_MECH_MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 4500;
    public static final double AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES = -50;
    public static final double AMP_MECH_IN_ANGLE = 160;
    public static final double AMP_MECH_ROLLER_SUCK_SPEED = 0.3;
    public static final double AMP_MECH_OUT_ANGLE = 45;
    public static final double AMP_MECH_DEPOSIT_SPEED = 0.6;
    public static final int BEAM_BREAK = 6;
    public static final double AMP_MECH_STOW_ANGLE = -100; //-120
    public static final double SUCK_IN_SPEED = -.1;

    public static final double SUCK_BACK_SPEED = -.4; //percent .set()
    public static final double AMP_MECH_OUT_ANGLE_TEST = 36;
    
}
