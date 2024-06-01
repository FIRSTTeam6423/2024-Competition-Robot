package frc.robot.subsystems.Drive.Module;

import static frc.robot.Constants.DriveConstants.kA;
import static frc.robot.Constants.DriveConstants.kV;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class ModuleIOSim extends ModuleIO {

  private final LinearSystemSim<N2, N1, N2> driveSim, pivotSim;

  public ModuleIOSim(String moduleID) {
    super(moduleID);

    driveSim =
        new LinearSystemSim<N2, N1, N2>(
            new LinearSystem<N2, N1, N2>(
                MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kV / kA),
                MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kA),
                MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
                MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)));

    pivotSim =
        new LinearSystemSim<N2, N1, N2>(
            new LinearSystem<N2, N1, N2>(
                MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kV / kA),
                MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kA),
                MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
                MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)));
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getDriveVelocity(), getPivotPose());
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDrivePose(), getPivotPose());
  }

  @Override
  public Rotation2d getPivotPose() {
    return Rotation2d.fromRadians(pivotSim.getOutput(0));
  }

  @Override
  public double getDrivePose() {
    return driveSim.getOutput(0);
  }

  @Override
  public double getDriveVelocity() {
    return driveSim.getOutput(1);
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotSim.setInput(voltage);
    pivotSim.update(0.020);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveSim.setInput(voltage);
    driveSim.update(0.020);
  }

  @Override
  public void resetEncoders() {}
}
