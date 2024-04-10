package frc.robot.AmpMech;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
}
