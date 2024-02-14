package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants{

   /**
     * DriveUtil Constants
     */
    public static final int FRONTLEFT_DRIVE = 1;
    public static final int FRONTLEFT_PIVOT = 2;
    public static final int FRONTRIGHT_DRIVE = 3;
    public static final int FRONTRIGHT_PIVOT = 4;
    public static final int BACKLEFT_DRIVE = 5;
    public static final int BACKLEFT_PIVOT = 6;
    public static final int BACKRIGHT_DRIVE = 7;
    public static final int BACKRIGHT_PIVOT = 8;
    
    public static final int FRONTLEFT_ABS_ENCODER = 0;
    public static final int FRONTRIGHT_ABS_ENCODER = 1;
    public static final int BACKLEFT_ABS_ENCODER = 2;
    public static final int BACKRIGHT_ABS_ENCODER = 3;



    /**
     * DriveUtil Constants
     */
    //public static final double WHEEL_RADIUS = 0.5;// its 2 inches?????
	  public static final double XBOX_STICK_DEADZONE_WIDTH = 0.05;
    public static final double MAX_ANGULAR_SPEED = 2500; //
    public static final double MAX_LINEAR_SPEED = 29.5; //meters per second
    
    //public static final double DRIVEPOSITIONCONVERSIONFACTOR = (1/7.13) * .096 * Math.PI;
    public static final double WHEEL_DIAMETER_INCHES=4;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI; //THIS IS EQUAL TO THE CIRCUMFERENCE OF THE WHEEL
    public static final double RPM_TO_METERS_PER_SEC = WHEEL_CIRCUMFERENCE_METERS/60;//default sparkmax velocity units is RPM so divide by 60

    
    public static final double MODULEDRIVE_P = 0.039753;//0.0024
    public static final double MODULEDRIVE_I = 0;
    public static final double MODULEDRIVE_D = 0;
    public static final double MODULEPIVOT_P = 0.005;//0.01;
    public static final double MODULEPIVOT_I = 0;
    public static final double MODULEPIVOT_D = 0;

    
    public static final double DEGREES_PER_ROTATION = 360;

    public static final double FRONTLEFT_ABS_ENCODER_OFFSET = 317.;
    public static final double FRONTRIGHT_ABS_ENCODER_OFFSET = 246;
    public static final double BACKLEFT_ABS_ENCODER_OFFSET = 236;
    public static final double BACKRIGHT_ABS_ENCODER_OFFSET = 275;

    public static final double[] ABS_ENCODER_OFFSETS = {
        FRONTLEFT_ABS_ENCODER_OFFSET,
        FRONTRIGHT_ABS_ENCODER_OFFSET,
        BACKLEFT_ABS_ENCODER_OFFSET,
        BACKRIGHT_ABS_ENCODER_OFFSET
    };

    //TODO: FIX THIS ITS PROLLY WRONG
    public static final double FRONTLEFT_X = 0.224;
    public static final double FRONTLEFT_Y = 0.224; //swap to negative
    public static final double FRONTLEFT_ANGLE = 45;
    public static final double FRONTRIGHT_X = 0.224;
    public static final double FRONTRIGHT_Y = -0.224; //swap to positive
    public static final double FRONTRIGHT_ANGLE = 315;
    public static final double BACKLEFT_X = -0.224;
    public static final double BACKLEFT_Y = 0.224; //swap to negative
    public static final double BACKLEFT_ANGLE = 135;
    public static final double BACKRIGHT_X = -0.224;
    public static final double BACKRIGHT_Y = -0.224; //swap to positve
    public static final double BACKRIGHT_ANGLE = 225;

    /**
     * Controller Input Device Mapping
     * 
     */
    public static final int XBOX_DRIVER = 1;
    public static final int XBOX_OPERATOR = 2;
    public static final int JOYSTICK_OPERATOR = 0;//1;
    
    public static final double ARM_JOYSTICK_INPUT_DEADBAND = .25;

    public static final Transform3d CAMERA_TO_ROBOT=new Transform3d(new Translation3d(-.0635, .1778, 0.0), new Rotation3d()); //Dummy
    
    //These are the April Tag coords for "Crescendo" (2024)
    public static final double APRIL1_X = Units.inchesToMeters(593.68);
    public static final double APRIL1_Y = Units.inchesToMeters(9.68);
    public static final double APRIL1_ROT = 120;
    public static final double APRIL2_X = Units.inchesToMeters(637.21);
    public static final double APRIL2_Y = Units.inchesToMeters(34.79);
    public static final double APRIL2_ROT = 120;
    public static final double APRIL3_X = Units.inchesToMeters(652.73);
    public static final double APRIL3_Y = Units.inchesToMeters(196.17);
    public static final double APRIL3_ROT = 180;
    public static final double APRIL4_X = Units.inchesToMeters(652.73);
    public static final double APRIL4_Y = Units.inchesToMeters(218.42);
    public static final double APRIL4_ROT = 180;
    public static final double APRIL5_X = Units.inchesToMeters(578.77);
    public static final double APRIL5_Y = Units.inchesToMeters(323.00);
    public static final double APRIL5_ROT = 270;
    public static final double APRIL6_X = Units.inchesToMeters(72.5);
    public static final double APRIL6_Y = Units.inchesToMeters(323.00);
    public static final double APRIL6_ROT = 270;
    public static final double APRIL7_X = Units.inchesToMeters(-1.50);
    public static final double APRIL7_Y = Units.inchesToMeters(218.42);
    public static final double APRIL7_ROT = 0;
    public static final double APRIL8_X = Units.inchesToMeters(-1.50);
    public static final double APRIL8_Y = Units.inchesToMeters(196.17);
    public static final double APRIL8_ROT = 0;
    public static final double APRIL9_X = Units.inchesToMeters(14.02);
    public static final double APRIL9_Y = Units.inchesToMeters(34.79);
    public static final double APRIL9_ROT = 60;
    public static final double APRIL10_X = Units.inchesToMeters(57.54);
    public static final double APRIL10_Y = Units.inchesToMeters(9.68);
    public static final double APRIL10_ROT = 60;
    public static final double APRIL11_X = Units.inchesToMeters(468.69);
    public static final double APRIL11_Y = Units.inchesToMeters(146.19);
    public static final double APRIL11_ROT = 300;
    public static final double APRIL12_X = Units.inchesToMeters(468.69);
    public static final double APRIL12_Y = Units.inchesToMeters(177.10);
    public static final double APRIL12_ROT = 60;
    public static final double APRIL13_X = Units.inchesToMeters(441.74);
    public static final double APRIL13_Y = Units.inchesToMeters(161.62);
    public static final double APRIL13_ROT = 180;
    public static final double APRIL14_X = Units.inchesToMeters(209.48);
    public static final double APRIL14_Y = Units.inchesToMeters(161.62);
    public static final double APRIL14_ROT = 0;
    public static final double APRIL15_X = Units.inchesToMeters(182.73);
    public static final double APRIL15_Y = Units.inchesToMeters(177.10);
    public static final double APRIL15_ROT = 120;
    public static final double APRIL16_X = Units.inchesToMeters(182.73);
    public static final double APRIL16_Y = Units.inchesToMeters(146.19);
    public static final double APRIL16_ROT = 240;
    
    // April Tag Heights
    public static final double APRIL1_Z = Units.inchesToMeters(53.38);
    public static final double APRIL2_Z = Units.inchesToMeters(53.38);
    public static final double APRIL3_Z = Units.inchesToMeters(57.13);
    public static final double APRIL4_Z = Units.inchesToMeters(57.13);
    public static final double APRIL5_Z = Units.inchesToMeters(53.38);
    public static final double APRIL6_Z = Units.inchesToMeters(53.38);
    public static final double APRIL7_Z = Units.inchesToMeters(57.13);
    public static final double APRIL8_Z = Units.inchesToMeters(57.13);
    public static final double APRIL9_Z = Units.inchesToMeters(53.38);
    public static final double APRIL10_Z = Units.inchesToMeters(53.38);
    public static final double APRIL11_Z = Units.inchesToMeters(52.00);
    public static final double APRIL12_Z = Units.inchesToMeters(52.00);
    public static final double APRIL13_Z = Units.inchesToMeters(52.00);
    public static final double APRIL14_Z = Units.inchesToMeters(52.00);
    public static final double APRIL15_Z = Units.inchesToMeters(52.00);
    public static final double APRIL16_Z = Units.inchesToMeters(52.00);

    public static final Pose3d[] TagPoses = {
            new Pose3d(APRIL1_X,APRIL1_Y,APRIL1_Z, new Rotation3d(0, 0, Math.toRadians(APRIL1_ROT))),
            new Pose3d(APRIL2_X,APRIL2_Y,APRIL2_Z, new Rotation3d(0, 0, Math.toRadians(APRIL2_ROT))),
            new Pose3d(APRIL3_X,APRIL3_Y,APRIL3_Z, new Rotation3d(0, 0, Math.toRadians(APRIL3_ROT))),
            new Pose3d(APRIL4_X,APRIL4_Y,APRIL4_Z,  new Rotation3d(0, 0, Math.toRadians(APRIL4_ROT))),
            new Pose3d(APRIL5_X,APRIL5_Y,APRIL5_Z,  new Rotation3d(0, 0, Math.toRadians(APRIL5_ROT))),
            new Pose3d(APRIL6_X,APRIL6_Y,APRIL6_Z, new Rotation3d(0, 0, Math.toRadians(APRIL6_ROT))),
            new Pose3d(APRIL7_X,APRIL7_Y,APRIL7_Z, new Rotation3d(0, 0, Math.toRadians(APRIL7_ROT))),
            new Pose3d(APRIL8_X,APRIL8_Y,APRIL8_Z, new Rotation3d(0, 0, Math.toRadians(APRIL8_ROT))),
            new Pose3d(APRIL9_X,APRIL9_Y,APRIL9_Z, new Rotation3d(0, 0, Math.toRadians(APRIL9_ROT))),
            new Pose3d(APRIL10_X,APRIL10_Y,APRIL10_Z, new Rotation3d(0, 0, Math.toRadians(APRIL10_ROT))),
            new Pose3d(APRIL11_X,APRIL11_Y,APRIL11_Z, new Rotation3d(0, 0, Math.toRadians(APRIL11_ROT))),
            new Pose3d(APRIL12_X,APRIL12_Y,APRIL12_Z,  new Rotation3d(0, 0, Math.toRadians(APRIL12_ROT))),
            new Pose3d(APRIL13_X,APRIL13_Y,APRIL13_Z,  new Rotation3d(0, 0, Math.toRadians(APRIL13_ROT))),
            new Pose3d(APRIL14_X,APRIL14_Y,APRIL14_Z, new Rotation3d(0, 0, Math.toRadians(APRIL14_ROT))),
            new Pose3d(APRIL15_X,APRIL15_Y,APRIL15_Z, new Rotation3d(0, 0, Math.toRadians(APRIL15_ROT))),
            new Pose3d(APRIL16_X,APRIL16_Y,APRIL16_Z, new Rotation3d(0, 0, Math.toRadians(APRIL16_ROT))),
        };
    // These are the April Tag coords for "Charged Up" (2023)
    /*
    public static final double APRIL1_X = 15.51;
        public static final double APRIL1_Y = 1.07;
        public static final double APRIL1_ROT = 180;
        public static final double APRIL2_X = 15.51;
        public static final double APRIL2_Y = 2.75;
        public static final double APRIL2_ROT = 180;
        public static final double APRIL3_X = 15.51;
        public static final double APRIL3_Y = 3.74;
        public static final double APRIL3_ROT = 180;
        public static final double APRIL4_X = 16.18;
        public static final double APRIL4_Y = 4.42;
        public static final double APRIL4_ROT = 180;
        public static final double APRIL5_X = 0.36;
        public static final double APRIL5_Y = 4.42;
        public static final double APRIL5_ROT = 0;
        public static final double APRIL6_X = 1.03;
        public static final double APRIL6_Y = 4.42;
        public static final double APRIL6_ROT = 0;
        public static final double APRIL7_X = 1.03;
        public static final double APRIL7_Y = 3.74;
        public static final double APRIL7_ROT = 0;
        public static final double APRIL8_X = 1.03;
        public static final double APRIL8_Y = 1.07;
        public static final double APRIL8_ROT = 0;
        public static final double GRID_TAG_HEIGHT = .36;//METERS /0.46; // Tags 1-3 (red) & 6-8 (blue)
        public static final double SUB_TAG_HEIGHT = .59;// 0.67; //Tags 4-5
        public static final Pose3d[] TagPoses = {
            new Pose3d(APRIL1_X,APRIL1_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL1_ROT))),
            new Pose3d(APRIL2_X,APRIL2_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL2_ROT))),
            new Pose3d(APRIL3_X,APRIL3_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL3_ROT))),
            new Pose3d(APRIL4_X,APRIL4_Y,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL4_ROT))),
            new Pose3d(APRIL5_X,APRIL5_Y,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL5_ROT))),
            new Pose3d(APRIL6_X,APRIL6_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL6_ROT))),
            new Pose3d(APRIL7_X,APRIL7_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL7_ROT))),
            new Pose3d(APRIL8_X,APRIL8_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL8_ROT))),
        };*/
}