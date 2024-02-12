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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoState;
import frc.robot.util.IronUtil;

public class CargoUtil extends SubsystemBase {
  /** Creates a new CargoUtil. */
  private CargoState cargoState;

  private CANSparkMax intakePivotMotor, ampMechPivotMotor, intakeRollerMotor, ampMechRollerMotor, shooterRollerMotor1, shooterRollerMotor2;

  private DutyCycleEncoder intakePivotEncoder, ampMechPivotEncoder;
  private RelativeEncoder shooterRollerEncoder1, shooterRollerEncoder2;

  private DigitalInput[] intakeLimitSwitches = {
    new DigitalInput(0), // TODO gotta fix outputs
    new DigitalInput(1),
    new DigitalInput(2),
    new DigitalInput(3),
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

    intakePivotProfile = new TrapezoidProfile(new Constraints(0, 0));
    intakePivotProfileGoal = new TrapezoidProfile.State();
    intakePivotProfileSetpointDeg = new TrapezoidProfile.State(intakePivotEncoder.getAbsolutePosition(), 0);
    
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
    ampMechPivotPIDController.enableContinuousInput(-360, 360);
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
    cargoState = state;
    profileTimer.reset();
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
      + Constants.AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES
    ).times(-1);
  }

  /**
   * Returns a Rotation2d of the amp mechanism angle relative to the horizontal, counter-clockwise. 
   */
  public Rotation2d getIntakeAngleRelativeToGround() {
    return Rotation2d.fromDegrees(
      intakePivotEncoder.getAbsolutePosition() * 360 
      + Constants.INTAKE_PIVOT_ENCODER_OFFSET
    );
  }

  public void operateCargoMachine(){
    
    switch(cargoState){
      case IDLE: //amp mech is in stow
        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);
      
        // setPivotMotor(
        //   intakePivotMotor, 
        //   intakePivotFeedForwardController, 
        //   intakePivotProfileSetpointDeg, 
        //   intakePivotPIDController, 
        //   getIntakeAngleRelativeToGround()
        // );

        setPivotMotor(
          ampMechPivotMotor, 
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
        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotOutGoalDeg);
        
        setPivotMotor(
          intakePivotMotor, 
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, 
          intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        ); //set the intake arm motor to lower to the ground

        intakeRollerMotor.set(Constants.INTAKE_ROLLER_SPEED); //start intake rollers to suck in note

        for(DigitalInput intakeSwitch: intakeLimitSwitches){
          if (intakeSwitch.get()){
            setState(CargoState.STOW);
          }
        }

        //TODO add sensing for note so we don't stow nothing, only stow if note detected.
        // if(driver.getBButton() == false){
        //   setState(CargoState.STOW);
        // }

        break;

      case STOW: //intake up, note inside
        intakeRollerMotor.set(0.0); //stop the intake rolling motors

        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);

        setPivotMotor(
          intakePivotMotor, 
          intakePivotFeedForwardController, 
          intakePivotProfileSetpointDeg, intakePivotPIDController, 
          getIntakeAngleRelativeToGround()
        );//lift intake to the stowing position

        // if (driver.getXButtonPressed() == true){
        //   cargoState = CargoState.SPINUP;
        // }

        // if (driver.getLeftBumperPressed() == true){ //check for amp mech handoff button
        //   cargoState = CargoState.HANDOFF;
        // }
        break; 

      case SPINUP: //everything stowed, shoot motors rev up

        //start spinning up motors
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );
        
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );
        
        // if (driver.getXButton() == false && shooterRollerEncoder1.getVelocity() == IronUtil.deadzone(Constants.SHOOTER_ROLLER_SPINUP_SPEED, 0)){
        //   setState(CargoState.SHOOT); 
        // } 
        break; 
      case SHOOT: //intake motors in reverse to push note into handoff & flywheels are at speed
        intakeRollerMotor.set(Constants.INTAKE_ROLLER_REVERSE_SPEED);
        if (shootTimer.get() >= Constants.SHOOTING_TIME)
        setState(CargoState.IDLE);
        break; 
      case HANDOFF: //intake motors moving out to push note to shooters, shooter moters moving slowly to push into handoff
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotHandoffGoalDeg);

        setPivotMotor(
          ampMechPivotMotor,
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
        
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotDepositGoalDeg);
        
        setPivotMotor(
          ampMechPivotMotor, 
          ampMechPivotFeedForwardController, 
          ampMechPivotProfileSetpointDeg, 
          ampMechPivotPIDController, 
          getAmpMechAngleRelativeToGround()
        ); //extend amp mech out again to be ready to deposit

        // if (driver.getLeftBumper() == false){setState(CargoState.DEPOSIT);}
        break; 

      case DEPOSIT: //amp mech takes note and moves it down
        ampMechRollerMotor.set(Constants.AMP_MECH_DEPOSIT_SPEED);
        setState(CargoState.IDLE);
        break;
    }
  }

  private void setPivotMotor(
    CANSparkMax motor, 
    ArmFeedforward feedForwardController, 
    TrapezoidProfile.State setPointDeg, 
    PIDController pidController, 
    Rotation2d curRot)
  {
    double output = MathUtil.clamp(
      feedForwardController.calculate(
        Math.toRadians(setPointDeg.position),
        0
        ) + pidController.calculate(
          curRot.getDegrees(), 
          setPointDeg.position
        ),
        -.4,
        .4
      );
    // motor.set(
    //   MathUtil.clamp(
    //   feedForwardController.calculate(
    //     Math.toRadians(setPointDeg.position),
    //     0
    //     ) + pidController.calculate(
    //       curRot.getDegrees(), 
    //       setPointDeg.position
    //     ),
    //     -.1,
    //     .1
    //   ) 
    // );
    SmartDashboard.putNumber("PID OUTPUT FOR PIVOT", output);
  }

  public void testIntakePivotToState(TrapezoidProfile.State goalState){
    intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, goalState);
    
    setPivotMotor(
      intakePivotMotor, 
      intakePivotFeedForwardController, 
      intakePivotProfileSetpointDeg, 
      intakePivotPIDController, 
      getIntakeAngleRelativeToGround()
    );
  }
  public void testAmpMechPivotToState(TrapezoidProfile.State goalState){
    ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, goalState);
    
    setPivotMotor(
      ampMechPivotMotor, 
      ampMechPivotFeedForwardController, 
      ampMechPivotProfileSetpointDeg, 
      ampMechPivotPIDController, 
      getAmpMechAngleRelativeToGround()
    );
  }

  public void testIntakeRollers(){
    intakeRollerMotor.set(Constants.INTAKE_ROLLER_SPEED);
  }
  
  public void testAmpMechRollers(){
    ampMechRollerMotor.set(Constants.AMP_MECH_ROLLER_SPEED);
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
    if (queueFire){
      if (IronUtil.deadzone(shooterRollerEncoder1.getVelocity()-Constants.SHOOTER_ROLLER_TARGET_VELOCITY, Constants.SHOOTER_ROLLER_TARGET_VELOCITY_ZONE) == 0
        && IronUtil.deadzone(shooterRollerEncoder1.getVelocity()-Constants.SHOOTER_ROLLER_TARGET_VELOCITY, Constants.SHOOTER_ROLLER_TARGET_VELOCITY_ZONE) == 0){
          setState(CargoState.SHOOT);
        }
    }

    //operateCargoMachine();

    SmartDashboard.putNumber("Intake pivot encoder value", getAmpMechAngleRelativeToGround().getDegrees());
    SmartDashboard.putNumber("Ampmech pivot encoder value", ampMechPivotEncoder.getAbsolutePosition()*360);
  }
}
