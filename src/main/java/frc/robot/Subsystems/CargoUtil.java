// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoState;

public class CargoUtil extends SubsystemBase {
  /** Creates a new CargoUtil. */
  private CargoState cargoState;

  private CANSparkMax intakePivotMotor, ampMechPivotMotor, intakeRollerMotor1, intakeRollerMotor2, ampMechRollerMotor;

  private DutyCycleEncoder intakePivotEncoder, ampMechPivotEncoder;
  
  private PIDController intakePivotPIDController, ampMechPivotPidController;
  private ArmFeedforward intakePivotFeedForwardController, ampMechPivotFeedForwardController;

  private TrapezoidProfile intakePivotProfile, ampMechPivotProfile;
  private TrapezoidProfile.State intakePivotProfileGoal, ampMechPivotProfileGoal;
  private TrapezoidProfile.State intakePivotProfileSetpoint, ampMechPivotSetpoint;

  public CargoUtil() {

    intakePivotProfile = new TrapezoidProfile(new Constraints(0, 0));
    intakePivotProfileGoal = new TrapezoidProfile.State();
    intakePivotProfileSetpoint = new TrapezoidProfile.State();

    ampMechPivotProfile = new TrapezoidProfile(new Constraints(0, 0));
    ampMechPivotProfileGoal = new TrapezoidProfile.State();
    ampMechPivotSetpoint = new TrapezoidProfile.State();
    
    cargoState = CargoState.IDLE;

    InitMotors();
    InitControlSystems();
  }

  private void InitControlSystems() {
    //=====control systems=====//
    intakePivotPIDController = new PIDController(Constants.INTAKE_PIVOT_P, Constants.INTAKE_PIVOT_I, Constants.INTAKE_PIVOT_D);
    ampMechPivotPidController = new PIDController(Constants.AMP_MECH_PIVOT_P, Constants.AMP_MECH_PIVOT_I, Constants.AMP_MECH_PIVOT_D);

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

    intakeRollerMotor1 = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR_1, MotorType.kBrushless);
    intakeRollerMotor2 = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR_2, MotorType.kBrushless);

    ampMechPivotMotor = new CANSparkMax(Constants.AMP_PIVOT_MOTOR, MotorType.kBrushless);
    ampMechRollerMotor = new CANSparkMax(Constants.AMP_ROLLER_MOTOR, MotorType.kBrushless);

    intakePivotMotor.setIdleMode(IdleMode.kBrake);

    intakeRollerMotor1.setIdleMode(IdleMode.kBrake);
    intakeRollerMotor2.setIdleMode(IdleMode.kBrake);
    
    ampMechPivotMotor.setIdleMode(IdleMode.kBrake);
    ampMechRollerMotor.setIdleMode(IdleMode.kBrake);

    intakeRollerMotor2.setInverted(true); //2 motors spin in opposite rotations

    intakePivotEncoder = new DutyCycleEncoder(Constants.INTAKE_PIVOT_ENCODER);
    ampMechPivotEncoder = new DutyCycleEncoder(Constants.AMP_PIVOT_ENCODER);
  }

  public void ChangeMachineState(){

    switch(cargoState){
      case IDLE: //amp mech is in stow
        break; 
      case INTAKING: //intake is down, rollers moving
        intakePivotProfileGoal = new TrapezoidProfile.State(5, 1); //move the intake down  or something
        break;
      case STOW: //intake up, note inside
        break; 
      case SPINUP: //everything stowed excet shoot motors
        break; 
      case SHOOT: //intake motors in reverse to push note into handoff & flywheels are at speed
        break; 
      case HANDOFF: //intake movers moving out to push note to shooters, shooter moters moving slowly to push into handoff
        break; 
      case DEPOSIT: //amp mech takes note and moves it down
        break;
    }

  }

  public void operateCargo(boolean intake){

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
