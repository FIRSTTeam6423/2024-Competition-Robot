package frc.robot.Climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.simWidgets.ClimbWidget;

public abstract class Climb extends SubsystemBase{
  
  private static final Climb instance;

  private final ClimbWidget climbWidget;

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

  public Climb() {
    climbWidget = new ClimbWidget();
  }

  public abstract double getSimLeftHeight();

  public abstract double getSimRightHeight();

  public abstract double getAverageCurrent();

  public abstract boolean atCurrentLimit();

  public abstract Command StopClimb();

  public abstract Command setVoltage(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier);

  @Override
  public void periodic() {
    climbWidget.setLeftHeight(getSimLeftHeight());
    climbWidget.setRightHeight(getSimRightHeight());
  }

}
