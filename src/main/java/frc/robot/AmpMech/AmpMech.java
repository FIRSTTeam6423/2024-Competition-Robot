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

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
    
}
