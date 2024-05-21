package frc.robot.subsystems.Climb;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbIOSim implements ClimbIO {

  private final DCMotorSim leftClimbMotor, rightClimbMotor;

  public ClimbIOSim() {
    leftClimbMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNEO(CLIMB_LEFT_MOTOR), GEARING);

    rightClimbMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getNEO(CLIMB_RIGHT_MOTOR), GEARING);
  }

  @Override
  public void updateInputs(final ClimbInputs inputs) {}

  @Override
  public Command stopClimb() {
    return Commands.runOnce(
        () -> {
          leftClimbMotor.setInputVoltage(0);
          rightClimbMotor.setInputVoltage(0);
        });
  }

  @Override
  public Command setVoltage(double leftVoltage, double rightVoltage) {
    return Commands.run(
        () -> {
          leftClimbMotor.setInputVoltage(leftVoltage);
          rightClimbMotor.setInputVoltage(rightVoltage);
        });
  }

  @Override
  public double getCurrent() {
    double output = leftClimbMotor.getCurrentDrawAmps() + rightClimbMotor.getCurrentDrawAmps();
    return output / 2;
  }
}
