package frc.robot.subsystems.Shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIOSim implements ShooterIO {

  private final DCMotorSim leftMotor, rightMotor;

  public ShooterIOSim() {
    leftMotor =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNEO(LEFT_MOTOR), 1);

    rightMotor =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNEO(RIGHT_MOTOR), 1);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    return;
  }

  @Override
  public Command setMotorVoltage(double leftVoltage, double rightVoltage) {
    return run(
        () -> {
          leftMotor.setInputVoltage(leftVoltage);
          rightMotor.setInputVoltage(rightVoltage);
        });
  }

  @Override
  public Command stopMotors() {
    return run(() -> setMotorVoltage(0, 0));
  }

  @Override
  public double getVelocityLeft() {
    return leftMotor.getAngularVelocityRPM();
  }

  @Override
  public double getVelocityRight() {
    return rightMotor.getAngularVelocityRPM();
  }
}
