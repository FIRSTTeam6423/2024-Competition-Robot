package frc.robot.Climb;

import java.util.function.Supplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimbSim extends Climb {
  
  private ElevatorSim simLeft;
  private ElevatorSim simRight;

  protected ClimbSim() {
    simLeft = new ElevatorSim(
      DCMotor.getNEO(ClimbConstants.CLIMB_LEFT_MOTOR),
      1, // ! IDK THE GEAR RATIO CHANGE LATER
      1, // ! wtf is the mass
      1, // ! wtf is the sprocket Radius
      1, // ! wtf is the min height
      1, // ! wtf is the max height
      true, // Simulate Gravity
      1 // wtf is the starting height (probably the same as min height)
    );

    simRight = new ElevatorSim(
      DCMotor.getNEO(ClimbConstants.CLIMB_RIGHT_MOTOR),
      1, // ! IDK THE GEAR RATIO CHANGE LATER
      1, // ! wtf is the mass
      1, // ! wtf is the sprocket Radius
      1, // ! wtf is the min height
      1, // ! wtf is the max height
      true, // Simulate Gravity
      1 // wtf is the starting height (probably the same as min height)
    );
  }

  @Override
  public double getAverageCurrent() {
    return (simLeft.getCurrentDrawAmps() + simRight.getCurrentDrawAmps())/2;
  }
  
  @Override
  public boolean atCurrentLimit() {
    return getAverageCurrent() > ClimbConstants.MAX_CURRENT_AMPS;
  }

  public void StopClimb() {
    simLeft.setInputVoltage(0.0);
    simRight.setInputVoltage(0.0);
  }

  public void setVoltage(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    double leftInput = leftSupplier.get();
    double rightInput = rightSupplier.get();
    double lmax = (leftInput < 0 ? ClimbConstants.MAX_RETRACT_VOLTAGE : ClimbConstants.MAX_EXTEND_VOLTAGE);
    double rmax = (rightInput < 0 ? ClimbConstants.MAX_RETRACT_VOLTAGE : ClimbConstants.MAX_EXTEND_VOLTAGE);

    simLeft.setInputVoltage(leftInput * lmax);
    simRight.setInputVoltage(rightInput * rmax);
  }
}
