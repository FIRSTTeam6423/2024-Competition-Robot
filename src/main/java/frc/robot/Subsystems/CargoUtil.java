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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CargoState;

public class CargoUtil extends SubsystemBase {
  /** Creates a new CargoUtil. */
  private CargoState cargoState;

  private CANSparkMax intakePivotMotor, ampMechPivotMotor, intakeRollerMotor, ampMechRollerMotor, shooterRollerMotor1, shooterRollerMotor2;

  private DutyCycleEncoder intakePivotEncoder, ampMechPivotEncoder;
  private RelativeEncoder shooterRollerEncoder1, shooterRollerEncoder2;
  
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

  private XboxController driver;

  public CargoUtil() {

    intakePivotOutGoalDeg = new TrapezoidProfile.State(Constants.INTAKE_PIVOT_OUT_ANGLE, 0);
    intakePivotInGoalDeg = new TrapezoidProfile.State(Constants.INTAKE_PIVOT_IN_ANGLE, 0);
  
    ampMechPivotDepositGoalDeg = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_DEPOSIT_ANGLE, 0);
    ampMechPivotHandoffGoalDeg = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_HANDOFF_ANGLE, 0);
    ampMechPivotIdleGoalDeg = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_IDLE_ANGLE, 0);

    intakePivotProfile = new TrapezoidProfile(new Constraints(0, 0));
    intakePivotProfileGoal = new TrapezoidProfile.State();
    intakePivotProfileSetpointDeg = new TrapezoidProfile.State();

    ampMechPivotProfile = new TrapezoidProfile(new Constraints(0, 0));
    ampMechPivotProfileGoal = new TrapezoidProfile.State();
    ampMechPivotProfileSetpointDeg = new TrapezoidProfile.State();
    
    cargoState = CargoState.IDLE;

    profileTimer = new Timer();
    shootTimer = new Timer();

    InitMotors();
    InitControlSystems();
  }

  private void InitControlSystems() {
    //=====control systems=====//
    intakePivotPIDController = new PIDController(Constants.INTAKE_PIVOT_P, Constants.INTAKE_PIVOT_I, Constants.INTAKE_PIVOT_D);
    ampMechPivotPIDController = new PIDController(Constants.AMP_MECH_PIVOT_P, Constants.AMP_MECH_PIVOT_I, Constants.AMP_MECH_PIVOT_D);
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

    shooterRollerMotor2.setInverted(true); //2 motors spin in opposite rotations

    intakePivotEncoder = new DutyCycleEncoder(Constants.INTAKE_PIVOT_ENCODER);
    ampMechPivotEncoder = new DutyCycleEncoder(Constants.AMP_MECH_PIVOT_ENCODER);

    shooterRollerEncoder1 = shooterRollerMotor1.getEncoder();
    shooterRollerEncoder2 = shooterRollerMotor2.getEncoder();
  }

  private void setState(CargoState state) {
    cargoState = state;
    profileTimer.reset();
  }

  /**
   * Returns a Rotation2d of the amp mechanism angle relative to the horizontal, counter-clockwise. 
   */
  private Rotation2d getAmpMechAngleRelativeToGround() {
    return Rotation2d.fromDegrees(
      ampMechPivotEncoder.getAbsolutePosition() * 360 
      + Constants.AMP_MECH_PIVOT_ENCODER_OFFSET
    );
  }

  /**
   * Returns a Rotation2d of the amp mechanism angle relative to the horizontal, counter-clockwise. 
   */
  private Rotation2d getIntakeAngleRelativeToGround() {
    return Rotation2d.fromDegrees(
      intakePivotEncoder.getAbsolutePosition() * 360 
      + Constants.INTAKE_PIVOT_ENCODER_OFFSET
    );
  }

  public void operateCargo(boolean intake){
    
    switch(cargoState){
      case IDLE: //amp mech is in stow
        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotIdleGoalDeg);
      
        intakePivotMotor.set(
          MathUtil.clamp( 
            intakePivotFeedForwardController.calculate(
              Math.toRadians(intakePivotProfileSetpointDeg.position),
              0
            ) + intakePivotPIDController.calculate(
              getIntakeAngleRelativeToGround().getDegrees(), 
              intakePivotProfileSetpointDeg.position
            ),
          -1,
          1
          )
        );
        ampMechPivotMotor.set(
          MathUtil.clamp( 
            ampMechPivotFeedForwardController.calculate(
              Math.toRadians(ampMechPivotProfileSetpointDeg.position),
              0
            ) + ampMechPivotPIDController.calculate(
              getAmpMechAngleRelativeToGround().getDegrees(), 
              ampMechPivotProfileSetpointDeg.position
            ),
          -1,
          1
          )
        );
        intakeRollerMotor.set(0.0);
        ampMechRollerMotor.set(0.0);
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), 0)
          );
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), 0)
          );

        if (driver.getBButtonPressed() == true){ //check for intaking button
          setState(CargoState.INTAKING);
        }
        
        break; 
      case INTAKING: //intake is down, rollers moving
       //move the intake down  or something
        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotOutGoalDeg);
        
        intakePivotMotor.set(
          MathUtil.clamp( 
            intakePivotFeedForwardController.calculate(
              Math.toRadians(intakePivotProfileSetpointDeg.position),
              0
            ) + intakePivotPIDController.calculate(
              getIntakeAngleRelativeToGround().getDegrees(), 
              intakePivotProfileSetpointDeg.position
            ),
            -1,
            1
          )
        ); //set the intake arm motor to lower to the ground

        intakeRollerMotor.set(Constants.INTAKE_ROLLER_SPEED); //start intake rollers to suck in note

        //TODO add sensing for note so we don't stow nothing, only stow if note detected.
        if(driver.getBButton() == false){
          setState(CargoState.STOW);
        }

        break;
      case STOW: //intake up, note inside
        intakeRollerMotor.set(0.0); //stop the intake rolling motors

        intakePivotProfileSetpointDeg = intakePivotProfile.calculate(profileTimer.get(), intakePivotProfileSetpointDeg, intakePivotInGoalDeg);

        intakePivotMotor.set(
          MathUtil.clamp( 
            intakePivotFeedForwardController.calculate(
              Math.toRadians(intakePivotProfileSetpointDeg.position),
              0
            ) + intakePivotPIDController.calculate(
              getIntakeAngleRelativeToGround().getDegrees(), 
              intakePivotProfileSetpointDeg.position
            ),
            -1,
            1
          )
        ); //lift intake to the stowing position

        if (driver.getXButtonPressed() == true){
          cargoState = CargoState.SPINUP;
        }

        if (driver.getLeftBumperPressed() == true){ //check for amp mech handoff button
          cargoState = CargoState.HANDOFF;
        }
        break; 
      case SPINUP: //everything stowed, shoot motors rev up

        //start spinning up motors
        shooterRollerMotor1.set(
          shooterRollerPIDController1.calculate(shooterRollerEncoder1.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );
        
        shooterRollerMotor2.set(
          shooterRollerPIDController2.calculate(shooterRollerEncoder2.getVelocity(), Constants.SHOOTER_ROLLER_SPINUP_SPEED)
        );
        
        if (driver.getXButton() == false 
          && shooterRollerEncoder1 == //Fred put the fix here <===================
        ) {
          setState(CargoState.SHOOT); 
        }
        break; 
      case SHOOT: //intake motors in reverse to push note into handoff & flywheels are at speed
        intakeRollerMotor.set(Constants.INTAKE_ROLLER_REVERSE_SPEED);
        if (shootTimer.get() >= Constants.SHOOTING_TIME)
        setState(CargoState.IDLE);
        break; 
      case HANDOFF: //intake motors moving out to push note to shooters, shooter moters moving slowly to push into handoff
        
        
        ampMechPivotProfileGoal = new TrapezoidProfile.State(Constants.AMP_MECH_PIVOT_HANDOFF_ANGLE, 0); //setting amp mech profiles
        ampMechPivotProfileSetpointDeg = ampMechPivotProfile.calculate(profileTimer.get(), ampMechPivotProfileSetpointDeg, ampMechPivotHandoffGoalDeg);
        
        ampMechPivotMotor.set(
          MathUtil.clamp(
            ampMechPivotFeedForwardController.calculate(
              Math.toRadians(ampMechPivotProfileSetpointDeg.position),
              0
            ) + ampMechPivotPIDController.calculate(
              ampMechPivotEncoder.getAbsolutePosition(), 
              ampMechPivotProfileSetpointDeg.position
            ),
            -1,
            1
          )
        ); //set the amp mech arm to go up to intake the note
        
        intakeRollerMotor.set(Constants.INTAKE_ROLLER_REVERSE_SPEED); //rolls the note into the shooter mech
        shooterRollerMotor1.set(shooterRollerPIDController1.calculate(shooterRollerEncoder1.getPosition(), Constants.SHOOTER_ROLLER_HANDOFF_SPEED)); //rolls the note
        shooterRollerMotor2.set(shooterRollerPIDController2.calculate(shooterRollerEncoder1.getPosition(), Constants.SHOOTER_ROLLER_HANDOFF_SPEED)); //up for the amp mech

        ampMechRollerMotor.set(Constants.AMP_MECH_ROLLER);

        break; 
      case DEPOSIT: //amp mech takes note and moves it down
        
        break;
    }

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
