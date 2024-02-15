// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.Subsystems.CargoUtil;
// import frc.robot.util.CargoState;

// public class HandleUserCargoInput extends Command {
//   private CargoUtil cu;


//   //UserOperaretCargo deals with input from the user, and calls setState accordingly to update the states
//   public HandleUserCargoInput(CargoUtil cu) {
//     addRequirements(cu);
//   }

//   @Override
//   public void initialize() {

//   }
//   @Override
//   public void execute() {
//     CargoState state = cu.getState();
//     switch (state) {
//       case IDLE:
//         if (RobotContainer.getDriverIntakeInput()) cu.setState(CargoState.INTAKING); //check for intaking button
//         //switch to next state (intaking) when some trigger is down
//         if (RobotContainer.getOperatorStowManualOverrideInput()) cu.setState(CargoState.STOW);
//         //TODO ADD DOUBLE TAP FOR MANUAL 
//         break;
//       case INTAKING:
//         //Intaking, if you release the intake button it should
//         //return to IDLE because it will automatically move to stow if a a note is detected
//         if (!RobotContainer.getDriverIntakeInput()) cu.setState(CargoState.IDLE);
//         //if intake button goes down
//         break;
//       case STOW:
//         if (RobotContainer.getOperatorSpinupInput()) cu.setState(CargoState.SPINUP);
//         if (RobotContainer.getOperatorHandoffInput()) cu.setState(CargoState.HANDOFF);
//         break;
//       case SPINUP:
//         if (RobotContainer.getDriverShootInput()) cu.requestFire();
//         break;
//       case HANDOFF:
//         if (!RobotContainer.getDriverDepositInput()) cu.setState(CargoState.DEPOSIT);
//         break;
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
