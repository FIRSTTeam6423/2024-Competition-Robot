package frc.robot.AmpMech;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AmpMechSim extends AmpMech {
  private final SingleJointedArmSim sim =
    new SingleJointedArmSim(
      LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(11),
        1, // ! PUT THE MOI HERE LATER 
        1 // ! PUT THE MOTOR GEARING LATER
      ),
      DCMotor.getNEO(11),
      1, // ! PUT THE MOTOR GEARING LATER
      1, // * Intake length prob
      1, // * Min angle Radians
      1, // * Max angle Radians
      true, // ! Simulates gravity
      1 // * Starting angle
    );

  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
    AmpMechConstants.AMP_MECH_PIVOT_kS, 
    AmpMechConstants.AMP_MECH_PIVOT_kG,
    AmpMechConstants.AMP_MECH_PIVOT_kV,
    AmpMechConstants.AMP_MECH_PIVOT_kA
  );

  private SimDeviceSim beamBreak = new SimDeviceSim(AmpMechConstants.BEAM_BREAK);

  @Override 
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    // double feedforward = pivotFeedForwardController.calculate(setpoint.position, setpoint.velocity);
    // pivotMotor.set(feedforward + output);
    double feedforward = pivotFeedForwardController.calculate(setpoint.position, setpoint.velocity);
    sim.setInputVoltage(feedforward + output);
 }
}
