package frc.robot.simWidgets;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot.Climb.Climb;

public class ClimbWidget {

  private final Mechanism2d climber;

  private final MechanismRoot2d climberRoot;
  private MechanismLigament2d climberLigament;

  public ClimbWidget() {
    climber = new Mechanism2d(10, 10);
    climberRoot = climber.getRoot("Climber root", 10, 0);
  }

  public void setLeftHeight(double newHeight) {
    /*double precent = newHeight / 4;

    double stageBottomY = 6 * precent;

    climberRoot.setPosition(leftRootX, stageBottomY);*/
  }

  public void setRightHeight(double newHeight) {
    /*double precent = newHeight / 4;

    double stageBottomY = 6 * precent;

    rightBottomRoot.setPosition(leftRootX, stageBottomY);*/
  }
}
