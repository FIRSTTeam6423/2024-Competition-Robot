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
import frc.robot.Constants;
import frc.robot.AmpMech.AmpMechConstants;
import frc.robot.Intake.IntakeConstants;

public class AmpMech extends ProfiledPIDSubsystem{

    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(AmpMechConstants.AMP_MECH_PIVOT_ENCODER);

    private CANSparkMax pivotMotor = new CANSparkMax(AmpMechConstants.AMP_MECH_PIVOT_MOTOR, MotorType.kBrushless);
    private CANSparkMax rollerMotor = new CANSparkMax(AmpMechConstants.AMP_MECH_ROLLER_MOTOR, MotorType.kBrushless);
    
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
    }

    private Rotation2d getAmpMechAngleRelativeToGround() {
        return Rotation2d.fromDegrees(
            pivotEncoder.getAbsolutePosition() * 360 
             ).times(-1).plus(Rotation2d.fromDegrees(AmpMechConstants.AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES));//-50
        }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = pivotFeedForwardController.calculate(setpoint.position, setpoint.velocity);
        //pivotMotor.set(feedforward + output);
        SmartDashboard.putNumber("Amp Mech Pivot out", feedforward + output);
    }

    @Override
    public double getMeasurement() {
        return getAmpMechAngleRelativeToGround().getDegrees();
    }

    public Command grabNote(){
        return this.runOnce(() ->{
            setGoal(AmpMechConstants.AMP_MECH_IN_ANGLE); // brings the amp mech up to accpet the note
            rollerMotor.set(AmpMechConstants.AMP_MECH_ROLLER_SUCK_SPEED);
        });
    }
    
    public Command extend(){
        return this.runOnce(() -> {
            setGoal(AmpMechConstants.AMP_MECH_OUT_ANGLE);
            rollerMotor.stopMotor();
        });
    }

    public Command deposit(){
        return this.runOnce(() -> {
            rollerMotor.set(AmpMechConstants.AMP_MECH_DEPOSIT_SPEED);
        });
    }
    
}
