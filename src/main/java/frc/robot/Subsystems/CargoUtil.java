// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.CargoState;
import frc.robot.util.IronUtil;

public class CargoUtil extends SubsystemBase {
  /** Creates a new CargoUtil. */
  private CargoState cargoState;

  private CANSparkMax intakePivotMotor, ampMechPivotMotor, intakeRollerMotor, ampMechRollerMotor, shooterRollerMotor1, shooterRollerMotor2;

  private DutyCycleEncoder intakePivotEncoder, ampMechPivotEncoder;
  private RelativeEncoder shooterRollerEncoder1, shooterRollerEncoder2;

  private DigitalInput[] intakeLimitSwitches = {
    new DigitalInput(7), // TODO gotta fix outputs
    new DigitalInput(8),
    new DigitalInput(9),
  };
  
  private PIDController intakePivotPIDController, ampMechPivotPIDController, shooterRollerPIDController1, shooterRollerPIDController2;
  private ArmFeedforward intakePivotFeedForwardController, ampMechPivotFeedForwardController;

  private TrapezoidProfile intakePivotProfile, ampMechPivotProfile;
  private TrapezoidProfile.State intakePivotProfileGoal, ampMechPivotProfileGoal;
  private TrapezoidProfile.State intakePivotProfileSetpointDeg, ampMechPivotProfileSetpointDeg;

  private TrapezoidProfile.State intakePivotOutGoalDeg;
  private TrapezoidProfile.State intakePivotInGoalDeg;

  private TrapezoidProfile.State ampMechPivotDepositGoalDeg;
  private TrapezoidProfile.State ampMechPivotHandoffGoalDeg;
  private TrapezoidProfile.State ampMechPivotIdleGoalDeg;

  private Timer profileTimer, shootTimer;

  private boolean queueFire;

  public CargoUtil() {
    InitMotors();
    InitControlSystems();
    intakePivotOutGoalDeg = new TrapezoidProfile.State(Constants.INTAKE_PIVOT_OUT_ANGLE, 0);
    intakePivotInGoalDeg = new TrapezoidProfile.State(Constants.INTAKE_PIVOT_IN_ANGLE, 0);
  
    ampMechPivotDepositGoalDeg = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_DEPOSIT_ANGLE, 0);
    ampMechPivotHandoffGoalDeg = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_HANDOFF_ANGLE, 0);
    ampMechPivotIdleGoalDeg = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_IDLE_ANGLE, 0);

    intakePivotProfile = new TrapezoidProfile(new Constraints(5, 1));
    intakePivotProfileGoal = new TrapezoidProfile.State();
    intakePivotProfileSetpointDeg = new TrapezoidProfile.State(getIntakeAngleRelativeToGround().getDegrees(), 0);
    
    ampMechPivotProfile = new TrapezoidProfile(new Constraints(0, 0));
    ampMechPivotProfileGoal = new TrapezoidProfile.State();
    ampMechPivotProfileSetpointDeg = new TrapezoidProfile.State();
    
    cargoState = CargoState.IDLE;

    profileTimer = new Timer();
    shootTimer = new Timer();


    
  }

  private void InitControlSystems() {
    //=====control systems=====//
    intakePivotPIDController = new PIDController(Constants.INTAKE_PIVOT_P, Constants.INTAKE_PIVOT_I, Constants.INTAKE_PIVOT_D);
    ampMechPivotPIDController = new PIDController(Constants.AMP_MECH_PIVOT_P, Constants.AMP_MECH_PIVOT_I, Constants.AMP_MECH_PIVOT_D);
    //ampMechPivotPIDController.enableContinuousInput(-180, 180);
    shooterRollerPIDController1 = new PIDController(Constants.SHOOTER_ROLLER_P, Constants.SHOOTER_ROLLER_I, Constants.SHOOTER_ROLLER_D);
    shooterRollerPIDController2 = new PIDController(Constants.SHOOTER_ROLLER_P, Constants.SHOOTER_ROLLER_I, Constants.SHOOTER_ROLLER_D);

    intakePivotFeedForwardController = new ArmFeedforward(
      Constants.INTAKE_PIVOT_kS, 
      Constants.INTAKE_PIVOT_kG,
      Constants.INTAKE_PIVOT_kV,
      Constants.INTAKE_PIVOT_kA
    );

    ampMechPivotFeedForwardController = new ArmFeedforward(
      Constants.AMP_MECH_PIVOT_kS, 
      Constants.AMP_MECH_PIVOT_kG,
      Constants.AMP_MECH_PIVOT_kV,
      Constants.AMP_MECH_PIVOT_kA
    );
  }

  public void InitMotors(){
    intakePivotMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    intakePivotMotor.setInverted(true);
    intakeRollerMotor = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR_1, MotorType.kBrushless);

    ampMechPivotMotor = new CANSparkMax(Constants.AMP_MECH_PIVOT_MOTOR, MotorType.kBrushless);
    ampMechRollerMotor = new CANSparkMax(Constants.AMP_MECH_ROLLER_MOTOR, MotorType.kBrushless);

    shooterRollerMotor1 = new CANSparkMax(Constants.SHOOTER_ROLLER_MOTOR1, MotorType.kBrushless);
    shooterRollerMotor2 = new CANSparkMax(Constants.SHOOTER_ROLLER_MOTOR2, MotorType.kBrushless);

    intakePivotMotor.setIdleMode(IdleMode.kBrake);

    intakeRollerMotor.setIdleMode(IdleMode.kBrake);
    
    ampMechPivotMotor.setIdleMode(IdleMode.kBrake);
    ampMechRollerMotor.setIdleMode(IdleMode.kBrake);

    shooterRollerMotor1.setInverted(true); //2 motors spin in opposite rotations
    shooterRollerMotor2.setInverted(false); //2 motors spin in opposite rotations

 
    intakePivotEncoder = new DutyCycleEncoder(Constants.INTAKE_PIVOT_ENCODER);
    ampMechPivotEncoder = new DutyCycleEncoder(Constants.AMP_MECH_PIVOT_ENCODER);

    shooterRollerEncoder1 = shooterRollerMotor1.getEncoder();
    shooterRollerEncoder2 = shooterRollerMotor2.getEncoder();
  }

  public void setState(CargoState state) {
    intakePivotProfileSetpointDeg = new TrapezoidProfile.State(getIntakeAngleRelativeToGround().getDegrees(), 0);
    cargoState = state;
    profileTimer.restart();
  }

  public void requestFire(){
    if (IronUtil.deadzone(shooterRollerEncoder1.getVelocity()-Constants.SHOOTER_ROLLER_TARGET_VELOCITY, Constants.SHOOTER_ROLLER_TARGET_VELOCITY_ZONE) == 0
     && IronUtil.deadzone(shooterRollerEncoder1.getVelocity()-Constants.SHOOTER_ROLLER_TARGET_VELOCITY, Constants.SHOOTER_ROLLER_TARGET_VELOCITY_ZONE) == 0){
      setState(CargoState.SHOOT);
    }
    else{
      queueFire = true;
    }
  }

  public CargoState getState() {
    return cargoState;
  }

  /**
   * Returns a Rotation2d of the amp mechanism angle relative to the horizontal, counter-clockwise. 
   */
  public Rotation2d getAmpMechAngleRelativeToGround() {
    return Rotation2d.fromDegrees(
      ampMechPivotEncoder.getAbsolutePosition() * 360 
    ).times(-1).plus(Rotation2d.fromDegrees(Constants.AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES));//-50
  }

  /**
   * Returns a Rotation2d of the amp mechanism angle relative to the horizontal, counter-clockwise. 
   */
  public Rotation2d getIntakeAngleRelativeToGround() {
     return Rotation2d.fromDegrees(
      intakePivotEncoder.getAbsolutePosition() * 360 
    ).plus(Rotation2d.fromDegrees(Constants.INTAKE_PIVOT_ENCODER_OFFSET_DEGREES));//-50
  }

  public void operateCargoMachine(){
    
    switch(cargoState){
      case IDLE: //amp mech is in stow
        RobotContainer.rumbleOperator(GenericHID.RumbleType.kBothRumble, 0);
        RobotContainer.rumbleDriver(GenericHID.RumbleType.kBothRumble, 0);


        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);
      
        setIntakePivotMotor(
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, 
          intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        );

        setAmpMechPivotMotor(
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        );

        intakeRollerMotor.set(0.0);
        ampMechRollerMotor.set(0.0);

        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), 0)
        );
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), 0)
        );

        //if (driver.getBButtonPressed() == true){setState(CargoState.INTAKING);} //check for intaking button
        break; 
        
      case INTAKING: //intake is down, rollers moving
       //move the intake down
        RobotContainer.rumbleDriver(GenericHID.RumbleType.kBothRumble, .25);

        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotOutGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);

        setIntakePivotMotor( 
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, 
          intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        ); //set the intake arm motor to lower to the ground

        setAmpMechPivotMotor(
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        );

        intakeRollerMotor.set(Constants.INTAKE_ROLLER_SPEED); //start intake rollers to suck in note

        for(DigitalInput intakeSwitch: intakeLimitSwitches){
          if (!intakeSwitch.get()){
            setState(CargoState.IDLE);
            break;
          }
        }

        //TODO add sensing for note so we don't stow nothing, only stow if note detected.
        // if(driver.getBButton() == false){
        //   setState(CargoState.STOW);
        // }

        break;

      case STOW: //intake up, note inside
        RobotContainer.rumbleOperator(GenericHID.RumbleType.kBothRumble, 0);
        RobotContainer.rumbleDriver(GenericHID.RumbleType.kBothRumble, 0);
        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);
      
        setIntakePivotMotor(
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, 
          intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        );

        setAmpMechPivotMotor(
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        );

        intakeRollerMotor.set(0.0);
        ampMechRollerMotor.set(0.0);
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), 0)
        );
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), 0)
        );
        break; 

      case SPINUP: //everything stowed, shoot motors rev up
        SmartDashboard.putBoolean("IN SHOOT STATE", false);
        RobotContainer.rumbleOperator(GenericHID.RumbleType.kBothRumble, 1);
        //start spinning up motors
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );
        
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );

        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);
      
        setIntakePivotMotor(
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, 
          intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        );

        setAmpMechPivotMotor(
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        );

        intakeRollerMotor.set(0.0);
        ampMechRollerMotor.set(0.0);
        shooterRollerMotor1.set(
          .5
          //shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), 0)
        );
        shooterRollerMotor2.set(
          .5
          //shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), 0)
        );
        break; 
      case SHOOT: //intake motors in reverse to push note into handoff & flywheels are at speed
        SmartDashboard.putBoolean("IN SHOOT STATE", true);
        RobotContainer.rumbleOperator(GenericHID.RumbleType.kBothRumble, 1);
        //start spinning up motors
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );
        
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );

        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);
      
        setIntakePivotMotor(
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, 
          intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        );

        setAmpMechPivotMotor(
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        );

        ampMechRollerMotor.set(0.0);
        intakeRollerMotor.set(Constants.INTAKE_ROLLER_REVERSE_SPEED);

        shooterRollerMotor1.set(
          .5
          //shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), 0)
        );
        shooterRollerMotor2.set(
          .5
          //shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), 0)
        );
        shootTimer.restart();
        break; 
      case HANDOFF: //intake motors moving out to push note to shooters, shooter moters moving slowly to push into handoff
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotHandoffGoalDeg);

        setAmpMechPivotMotor(
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        );//set the amp mech arm to go up to intake the note
        
        intakeRollerMotor.set(Constants.INTAKE_ROLLER_REVERSE_SPEED); //rolls the note into the shooter mech
        
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_HANDOFF_SPEED)
        ); //rolls the note
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_HANDOFF_SPEED)
        ); //up for the amp mech

        ampMechRollerMotor.set(Constants.AMP_MECH_ROLLER_SPEED); //amp mech takes in note
        

        // if (driver.getLeftBumper() == false){setState(CargoState.DEPOSIT);}
        break; 

      case DEPOSIT: //amp mech takes note and moves it down
        ampMechRollerMotor.set(Constants.AMP_MECH_DEPOSIT_SPEED);
        //move amp mech to deposit
        setState(CargoState.IDLE);
        break;
    }
  }

  private void setAmpMechPivotMotor(
    ArmFeedforward feedForwardController,
    TrapezoidProfile.State setpointDeg,
    PIDController pidController,
    Rotation2d curRot) 
    {
      double output = MathUtil.clamp(
      feedForwardController.calculate(
        Math.toRadians(setpointDeg.position),
        0
        ) + pidController.calculate(
          curRot.getDegrees(), 
          setpointDeg.position
        ),
        -.5,
        .5
      );
    ampMechPivotMotor.set(
      output
    );
    SmartDashboard.putNumber("AMP PIVOT OUT: ", output);
  }

  private void setIntakePivotMotor(
    ArmFeedforward feedForwardController, 
    TrapezoidProfile.State setpointDeg, 
    PIDController pidController, 
    Rotation2d curRot)
  {
    double output = MathUtil.clamp(
      feedForwardController.calculate(
        Math.toRadians(setpointDeg.position),
        0
        ) + pidController.calculate(
          curRot.getDegrees(), 
          setpointDeg.position
        ),
        -.5,
        .5
      );
    intakePivotMotor.set(
      output
      //.1
    );
    SmartDashboard.putNumber("Intake pivot CURRENT", intakePivotMotor.getOutputCurrent());
    System.out.println("INTAKE PIVOT CURRENT: " + intakePivotMotor.getOutputCurrent());
    //SmartDashboard.putNumber("INTAKE PIVOT OUT: ", output);
  }

  public void testIntakeRollers(){
    intakeRollerMotor.set(Constants.INTAKE_ROLLER_SPEED);
  }
  
  public void testAmpMechRollers(){
    ampMechRollerMotor.set(Constants.AMP_MECH_ROLLER_SPEED);
  }

  public boolean ampAtSetpoint() {
    return IronUtil.inRange(getAmpMechAngleRelativeToGround().getDegrees(), ampMechPivotProfileSetpointDeg.position, Constants.AMP_MECH_PIVOT_DEADBAND_DEGREES);
  }

  public boolean intakeAtSetpoint() {
    return IronUtil.inRange(getIntakeAngleRelativeToGround().getDegrees(), intakePivotProfileSetpointDeg.position, Constants.INTAKE_PIVOT_DEADBAND_DEGREES);
  }

  public void testShooterRollers(){
    /*shooterRollerMotor1.set(
      shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_HANDOFF_SPEED)
    );
    shooterRollerMotor2.set(
      shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), Constants.SHOOTER_ROLLER_HANDOFF_SPEED)
    );*/
    shooterRollerMotor1.setVoltage(10);
    shooterRollerMotor2.setVoltage(10);
  }
  public void resetProfileTimer(){
    profileTimer.reset();
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("SWTHC 1", intakeLimitSwitches[0].get());
    SmartDashboard.putBoolean("SWTHC 2", intakeLimitSwitches[1].get());
    SmartDashboard.putBoolean("SWTHC 3", intakeLimitSwitches[2].get());

    if (queueFire){
      if (IronUtil.deadzone(shooterRollerEncoder1.getVelocity()-Constants.SHOOTER_ROLLER_TARGET_VELOCITY, Constants.SHOOTER_ROLLER_TARGET_VELOCITY_ZONE) == 0
        && IronUtil.deadzone(shooterRollerEncoder1.getVelocity()-Constants.SHOOTER_ROLLER_TARGET_VELOCITY, Constants.SHOOTER_ROLLER_TARGET_VELOCITY_ZONE) == 0){
          setState(CargoState.SHOOT);
        }
    }

    //operateCargoMachine();

    SmartDashboard.putNumber("Ampmech pivot encoder value", getAmpMechAngleRelativeToGround().getDegrees());
    SmartDashboard.putNumber("Intake pivot encoder value", getIntakeAngleRelativeToGround().getDegrees());
  }
}
