package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public abstract class Shooter extends SubsystemBase{

  private static final Shooter instance;

  static {
    if ( Robot.isReal() ) {
      instance = new ShooterReal();
    } else {
      instance = new ShooterSim();
    }
  }

  public static Shooter getInstance() {
    return instance;
  }

  public Shooter() {}

  public abstract double getMeasurementLeft();

  public abstract double getMeasurementRight();

  public abstract void useOutputLeft(double output);

  public abstract void useOutputRight(double output);

  public abstract boolean atRPM();

  public abstract Command spinup();

  public abstract void setGoal(double newGoal); 

  public abstract Command startSpinup();

  public abstract Command stopRollers();

  public abstract Boolean rightTriggerPressed();

  public abstract Command feed();

  public abstract Command suckIn();

  public abstract Command suckBack();

}
