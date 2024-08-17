// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.IronUtil.IronController;
import frc.robot.subsystems.Drive.Drive;

public class DriveJoystick {
  private final Drive drive;
  private final IronController controller;
  private final double maxLinearSpeed, maxRotationSpeed;

  public DriveJoystick(
      Drive drive, IronController controller, double maxLinearSpeed, double maxRotationSpeed) {
    this.drive = drive;
    this.controller = controller;
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxRotationSpeed = maxRotationSpeed;
  }

  public Command execute() {
    return drive.drive(
        () ->
            DriverStation.isAutonomous()
                ? 0
                : -controller.joystickDeadbandOutput(XboxController.Axis.kLeftX.value)
                    * ((controller.getLeftTriggerAxis() > controller.axisDeadband)
                        ? maxLinearSpeed * .35
                        : maxLinearSpeed),
        () ->
            DriverStation.isAutonomous()
                ? 0
                : -controller.joystickDeadbandOutput(XboxController.Axis.kLeftY.value)
                    * ((controller.getLeftTriggerAxis() > controller.axisDeadband)
                        ? maxLinearSpeed * .35
                        : maxLinearSpeed),
        () ->
            controller.flickStickOutput(
                XboxController.Axis.kRightX.value, XboxController.Axis.kRightY.value)
        // () -> controller.joystickDeadbandOutput(XboxController.Axis.kRightX.value) *
        // maxRotationSpeed
        );
  }
}
