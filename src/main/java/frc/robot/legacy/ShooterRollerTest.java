// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.CargoUtil;

// public class ShooterRollerTest extends Command {
//   /** Creates a new ShooterRollerTest. */

//   CargoUtil cu;

//   //private Timer timer;

//   public ShooterRollerTest(CargoUtil cu) {
//     this.cu = cu;
//     addRequirements(cu);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     //timer.reset(); //wait idk if i need to use restart of reset i think we need to use restart tbh
//   }                //for not just this for everything because resetting stops the timer after but restarting starts  it again
  

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     cu.testShooterRollers();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     //return timer.get() > 4;
//     return false;
//   }
// }
