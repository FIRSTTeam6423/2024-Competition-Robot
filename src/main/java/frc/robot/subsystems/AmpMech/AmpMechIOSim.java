package frc.robot.subsystems.AmpMech;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.AmpMech.AmpMechConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpMechIOSim implements AmpMechIO {

    private final DCMotorSim rollerMotor, pivotMotor;

    public AmpMechIOSim() {
        rollerMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                0.1, 
                0.1
            ), 
            DCMotor.getNEO(AMP_MECH_ROLLER_MOTOR), 
            1
        );

        pivotMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                0.1, 
                0.1
            ), 
            DCMotor.getNEO(AMP_MECH_PIVOT_MOTOR), 
            1
        );
    }

    @Override
    public void updateInputs(final AmpMechInputs inputs) {}

    @Override
    public Rotation2d getAmpMechAngleRelativeToGround() {
        return new Rotation2d();
    }

    @Override
    public Command setRollerSpeed(double speed) {
        return run( () -> {
            rollerMotor.setInputVoltage(speed);
        });
    }

    @Override
    public Command stopRoller() {
        return run( () -> {
            rollerMotor.setInputVoltage(0);
        });
    }

    @Override
    public Command setPivotSpeed(double speed) {
        return run( () -> {
            pivotMotor.setInput(speed);
        });
    }

    @Override
    public Rotation2d getPivotPosition() {
        return Rotation2d.fromRotations(
            pivotMotor.getAngularPositionRotations()
        );
    }

    @Override
    public boolean getBeambreakStatus() {
        return true;
    }

}