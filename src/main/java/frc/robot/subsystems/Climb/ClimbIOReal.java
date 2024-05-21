package frc.robot.subsystems.Climb;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbIOReal implements ClimbIO {

  private final CANSparkMax leftClimbMotor, rightClimbMotor;

  public ClimbIOReal() {
    leftClimbMotor = new CANSparkMax(CLIMB_LEFT_MOTOR, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(CLIMB_RIGHT_MOTOR, MotorType.kBrushless);

    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(final ClimbInputs inputs) {
    inputs.averageCurrent = getCurrent();
    inputs.leftRotations = 0.0;
    inputs.rightRotations = 0.0;
    inputs.appliedVoltageLeft = leftClimbMotor.getBusVoltage();
    inputs.appliedVoltageRight = rightClimbMotor.getBusVoltage();
  }

  @Override
  public Command setVoltage(double leftVoltage, double rightVoltage) {
    return Commands.run(
        () -> {
          leftClimbMotor.setVoltage(leftVoltage);
          rightClimbMotor.setVoltage(rightVoltage);
        });
  }

  @Override
  public Command stopClimb() {
    return Commands.runOnce(
        () -> {
          leftClimbMotor.stopMotor();
          rightClimbMotor.stopMotor();
        });
  }

  @Override
  public double getCurrent() {
    double output = leftClimbMotor.getOutputCurrent() + rightClimbMotor.getOutputCurrent();
    return output / 2;
  }
}
