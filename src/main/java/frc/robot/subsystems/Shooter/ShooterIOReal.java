package frc.robot.subsystems.Shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIOReal implements ShooterIO {

  private final CANSparkMax leftMotor, rightMotor;
  private final RelativeEncoder leftEncoder, rightEncoder;

  public ShooterIOReal() {
    leftMotor = new CANSparkMax(LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RIGHT_MOTOR, MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.leftVelocity = getVelocityLeft();
    inputs.rightVelocity = getVelocityRight();
    inputs.leftAppliedCurrent = leftMotor.getOutputCurrent();
    inputs.rightAppliedCurrent = rightMotor.getOutputCurrent();
    inputs.leftAppliedVoltage = leftMotor.getBusVoltage();
    inputs.rightAppliedVoltage = rightMotor.getBusVoltage();
  }

  @Override
  public Command setMotorVoltage(double leftVoltage, double rightVoltage) {
    return run(
        () -> {
          leftMotor.setVoltage(leftVoltage);
          rightMotor.setVoltage(rightVoltage);
        });
  }

  @Override
  public Command stopMotors() {
    return run(
        () -> {
          leftMotor.stopMotor();
          rightMotor.stopMotor();
        });
  }

  @Override
  public double getVelocityLeft() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getVelocityRight() {
    return rightEncoder.getVelocity();
  }
}
