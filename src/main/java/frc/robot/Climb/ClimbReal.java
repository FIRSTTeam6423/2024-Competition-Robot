package frc.robot.Climb;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbReal extends Climb {
  
  private final CANSparkMax leftClimb, rightClimb;

  protected ClimbReal() {
    rightClimb = new CANSparkMax(ClimbConstants.CLIMB_RIGHT_MOTOR, MotorType.kBrushless);
    
    leftClimb = new CANSparkMax(ClimbConstants.CLIMB_LEFT_MOTOR, MotorType.kBrushless);

    leftClimb.setInverted(true);
    rightClimb.setInverted(false);

    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);
  }

  
  @Override
  public double getSimRightHeight() {return 1;}

  @Override 
  public double getSimLeftHeight() {return 1;}

  @Override
  public double getAverageCurrent() {
    return (leftClimb.getOutputCurrent() + rightClimb.getOutputCurrent()) / 2;
  }

  @Override
  public boolean atCurrentLimit() {
    return getAverageCurrent() > ClimbConstants.MAX_CURRENT_AMPS;
  }

  @Override
  public Command StopClimb() {
    return Commands.runOnce(() -> {
      leftClimb.stopMotor();
      rightClimb.stopMotor();
    });
  }

  @Override
  public Command setVoltage(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    return Commands.run(() -> {
      double leftInput = leftSupplier.get();
      double rightInput = rightSupplier.get();
      double lmax = (leftInput < 0 ? ClimbConstants.MAX_RETRACT_VOLTAGE : ClimbConstants.MAX_EXTEND_VOLTAGE);
      double rmax = (rightInput < 0 ? ClimbConstants.MAX_RETRACT_VOLTAGE : ClimbConstants.MAX_EXTEND_VOLTAGE); 
      leftClimb.setVoltage(leftInput * lmax);
      rightClimb.setVoltage(rightInput * rmax);
    });
  }

}
