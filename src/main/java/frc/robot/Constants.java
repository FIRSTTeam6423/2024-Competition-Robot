package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
    
    public static final int TOPLEFT_ABS_ENCODER = 0;
    public static final int TOPRIGHT_ABS_ENCODER = 1;
    public static final int BOTTOMLEFT_ABS_ENCODER = 2;
    public static final int BOTTOMRIGHT_ABS_ENCODER = 3;



    /**
     * DriveUtil Constants
     */
    public static final double WHEEL_RADIUS = 0.5;
	public static final double XBOX_STICK_DEADZONE_WIDTH = 0.05;
    public static final double MAX_ANGULAR_SPEED = 2500; //
    public static final double MAX_LINEAR_SPEED = 29.5; //meters per second
    
    public static final double DRIVECONVERSIONFACTOR = (1/7.13) * .096 * Math.PI;

    
    public static final double MODULEDRIVE_P = 0.039753;//0.0024
    public static final double MODULEDRIVE_I = 0;
    public static final double MODULEDRIVE_D = 0;
    public static final double MODULEPIVOT_P = 0.005;//0.01;
    public static final double MODULEPIVOT_I = 0;
    public static final double MODULEPIVOT_D = 0;

    
    public static final double DEGREES_PER_ROTATION = 360;

    public static final double TOPLEFT_ABS_ENCODER_OFFSET = 116.233;
    public static final double TOPRIGHT_ABS_ENCODER_OFFSET = 77.01;
    public static final double BOTTOMLEFT_ABS_ENCODER_OFFSET = 73.7016 + 180;
    public static final double BOTTOMRIGHT_ABS_ENCODER_OFFSET = 131.52;

    public static final double[] ABS_ENCODER_OFFSETS = {
        TOPLEFT_ABS_ENCODER_OFFSET,
        TOPRIGHT_ABS_ENCODER_OFFSET,
        BOTTOMLEFT_ABS_ENCODER_OFFSET,
        BOTTOMRIGHT_ABS_ENCODER_OFFSET
    };

    public static final double TOPLEFT_X = 0.224;
    public static final double TOPLEFT_Y = 0.224; //swap to negative
    public static final double TOPLEFT_ANGLE = 45;
    public static final double TOPRIGHT_X = 0.224;
    public static final double TOPRIGHT_Y = -0.224; //swap to positive
    public static final double TOPRIGHT_ANGLE = 315;
    public static final double BOTTOMLEFT_X = -0.224;
    public static final double BOTTOMLEFT_Y = 0.224; //swap to negative
    public static final double BOTTOMLEFT_ANGLE = 135;
    public static final double BOTTOMRIGHT_X = -0.224;
    public static final double BOTTOMRIGHT_Y = -0.224; //swap to positve
    public static final double BOTTOMRIGHT_ANGLE = 225;

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
    public static final double APRIL1_X = 593.68;
    public static final double APRIL1_Y = 9.68;
    public static final double APRIL1_ROT = 120;
    public static final double APRIL2_X = 637.21;
    public static final double APRIL2_Y = 34.79;
    public static final double APRIL2_ROT = 120;
    public static final double APRIL3_X = 652.73;
    public static final double APRIL3_Y = 196.17;
    public static final double APRIL3_ROT = 180
    public static final double APRIL4_X = 652.73;
    public static final double APRIL4_Y = 218.42;
    public static final double APRIL4_ROT = 180;
    public static final double APRIL5_X = 578.77;
    public static final double APRIL5_Y = 323.00;
    public static final double APRIL5_ROT = 270;
    public static final double APRIL6_X = 72.5;
    public static final double APRIL6_Y = 323.00;
    public static final double APRIL6_ROT = 270;
    public static final double APRIL7_X = -1.50;
    public static final double APRIL7_Y = 218.42;
    public static final double APRIL7_ROT = 0;
    public static final double APRIL8_X = -1.50;
    public static final double APRIL8_Y = 196.17;
    public static final double APRIL8_ROT = 0;
    public static final double APRIL9_X = 14.02;
    public static final double APRIL9_Y = 34.79;
    public static final double APRIL9_ROT = 60;
    public static final double APRIL10_X = 57.54;
    public static final double APRIL10_Y = 9.68;
    public static final double APRIL10_ROT = 60;
    public static final double APRIL11_X = 468.69;
    public static final double APRIL11_Y = 146.19;
    public static final double APRIL11_ROT = 300;
    public static final double APRIL12_X = 468.69;
    public static final double APRIL12_Y =177.10;
    public static final double APRIL12_ROT = 60;
    public static final double APRIL13_X = 441.74;
    public static final double APRIL13_Y = 161.62;
    public static final double APRIL13_ROT = 180;
    public static final double APRIL14_X = 209.48;
    public static final double APRIL14_Y = 161.62;
    public static final double APRIL14_ROT = 0;
    public static final double APRIL15_X = 182.73;
    public static final double APRIL15_Y = 177.10;
    public static final double APRIL15_ROT = 120;
    public static final double APRIL16_X = 182.73;
    public static final double APRIL16_Y = 146.19;
    public static final double APRIL16_ROT = 240;
    public static final double GRID_TAG_HEIGHT = .36;//METERS /0.46; // Tags 1-3 (red) & 6-8 (blue)
    public static final double SUB_TAG_HEIGHT = .59;// 0.67; //Tags 4-5
    // The *0.0254 is used to convert the coords which are in inches to meters
    public static final Pose3d[] TagPoses = {
            new Pose3d(APRIL1_X*0.0254,APRIL1_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL1_ROT))),
            new Pose3d(APRIL2_X*0.0254,APRIL2_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL2_ROT))),
            new Pose3d(APRIL3_X*0.0254,APRIL3_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL3_ROT))),
            new Pose3d(APRIL4_X*0.0254,APRIL4_Y*0.0254,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL4_ROT))),
            new Pose3d(APRIL5_X*0.0254,APRIL5_Y*0.0254,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL5_ROT))),
            new Pose3d(APRIL6_X*0.0254,APRIL6_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL6_ROT))),
            new Pose3d(APRIL7_X*0.0254,APRIL7_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL7_ROT))),
            new Pose3d(APRIL8_X*0.0254,APRIL8_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL8_ROT))),
            new Pose3d(APRIL9_X*0.0254,APRIL9_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL9_ROT))),
            new Pose3d(APRIL10_X*0.0254,APRIL10_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL10_ROT))),
            new Pose3d(APRIL11_X*0.0254,APRIL11_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL11_ROT))),
            new Pose3d(APRIL12_X*0.0254,APRIL12_Y*0.0254,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL12_ROT))),
            new Pose3d(APRIL13_X*0.0254,APRIL13_Y*0.0254,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL13_ROT))),
            new Pose3d(APRIL14_X*0.0254,APRIL14_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL14_ROT))),
            new Pose3d(APRIL15_X*0.0254,APRIL15_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL15_ROT))),
            new Pose3d(APRIL16_X*0.0254,APRIL16_Y*0.0254,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL16_ROT))),
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