package frc.robot.AmpMech;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.AmpMech.AmpMechConstants;
import frc.robot.Intake.IntakeConstants;
import frc.robot.commons.IronUtil;

public class AmpMech extends ProfiledPIDSubsystem{

    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(AmpMechConstants.AMP_MECH_PIVOT_ENCODER);

    private CANSparkMax pivotMotor = new CANSparkMax(AmpMechConstants.AMP_MECH_PIVOT_MOTOR, MotorType.kBrushless);
    private CANSparkMax rollerMotor = new CANSparkMax(AmpMechConstants.AMP_MECH_ROLLER_MOTOR, MotorType.kBrushless);
    
    private DigitalInput beamBreak = new DigitalInput(AmpMechConstants.BEAM_BREAK);

    private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
      AmpMechConstants.AMP_MECH_PIVOT_kS, 
      AmpMechConstants.AMP_MECH_PIVOT_kG,
      AmpMechConstants.AMP_MECH_PIVOT_kV,
      AmpMechConstants.AMP_MECH_PIVOT_kA
    );
    
    public AmpMech() {
        super(
            new ProfiledPIDController(
                AmpMechConstants.AMP_MECH_PIVOT_P,
                AmpMechConstants.AMP_MECH_PIVOT_I,
                AmpMechConstants.AMP_MECH_PIVOT_D,
                new TrapezoidProfile.Constraints(
                    AmpMechConstants.AMP_MECH_MAX_VELOCITY_DEG_PER_SEC,
                    AmpMechConstants.AMP_MECH_MAX_ACCELERATION_DEG_PER_SEC_SQUARED
                )
            ),
            0
        );

        this.getController().setTolerance(10);
    }

    public boolean atGoal(){
        return this.getController().atGoal();
    }

    public boolean beamBreakHit(){
        if(!beamBreak.get()) System.out.println("LJDSLJFYALKSUHDFS");
        return !beamBreak.get();
    }

    private Rotation2d getAmpMechAngleRelativeToGround() {
        return Rotation2d.fromDegrees(
            pivotEncoder.getAbsolutePosition() * 360 
             ).times(-1).plus(Rotation2d.fromDegrees(AmpMechConstants.AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES));//-50
        }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = pivotFeedForwardController.calculate(setpoint.position, setpoint.velocity);
        pivotMotor.set(feedforward + output);
        SmartDashboard.putNumber("Amp Mech Pivot pos", getAmpMechAngleRelativeToGround().getDegrees());
        SmartDashboard.putNumber("AMP MECH Setpoint", setpoint.position);
        SmartDashboard.putBoolean("Beambreak", beamBreak.get());
    }

    @Override
    public double getMeasurement() {
        return getAmpMechAngleRelativeToGround().getDegrees();
    }

    public Command prepareGrab(){
        return this.runOnce(() ->{
            enable();
            setGoal(AmpMechConstants.AMP_MECH_IN_ANGLE); // brings the amp mech up to accpet the note
            //rollerMotor.set(AmpMechConstants.AMP_MECH_ROLLER_SUCK_SPEED);
        });
    }
    
    public Command suckNote(){
        return this.run(() ->{
            rollerMotor.set(AmpMechConstants.AMP_MECH_ROLLER_SUCK_SPEED);   
        });
    }
    
    public Command extend(){
        return this.runOnce(() -> {
            enable();
            //System.out.println("in extend");
            setGoal(AmpMechConstants.AMP_MECH_OUT_ANGLE);
        });
    }

    public Command deposit(){
        return this.run(() -> {
            rollerMotor.set(AmpMechConstants.AMP_MECH_DEPOSIT_SPEED);
        });
    }

    public Command stopRollers(){
        return this.runOnce(() ->{
            rollerMotor.stopMotor();
        });
    }

    public Command stow() {
        return this.runOnce(() ->{
            enable();
            setGoal(AmpMechConstants.AMP_MECH_STOW_ANGLE);
        });
    }

    public Command waitUntilBeamBreakIs(boolean cond) {        
    return new WaitUntilCommand(() -> this.beamBreakHit() == cond);
  }

  public Command suckIn(){
    return this.run(()->{
        rollerMotor.set(AmpMechConstants.SUCK_IN_SPEED);
    });
  }
}
