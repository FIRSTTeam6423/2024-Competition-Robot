package frc.robot.Climb;

import java.awt.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Climb.ClimbConstants;

public abstract class Climb extends SubsystemBase{
  
  private static final Climb instance;

  static {
    if ( Robot.isReal() ) {
      instance = new ClimbReal(); 
    } else {
      instance = new ClimbSim();
    }
  }

  public static Climb getInstance() {
    return instance;
  }

  public Climb() {}

  public abstract double getAverageCurrent();

  public abstract boolean atCurrentLimit();

  public abstract void StopClimb();

  public abstract void setVoltage(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier);

}
