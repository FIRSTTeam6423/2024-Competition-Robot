// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.CargoUtil;

// public class IntakeRollerTest extends Command {

//   private Timer timer;

//   public CargoUtil cu;
//   /** Creates a new IntakeRollerTest. */
//   public IntakeRollerTest(CargoUtil cu) {
//     this.cu = cu;
//     addRequirements(cu);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     cu.testIntakeRollers();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.get()>2;
//   }
// }