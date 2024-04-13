package frc.robot.simWidgets;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbWidget {

  private final Mechanism2d climber;

  public ClimbWidget() {
    climber = new Mechanism2d(10, 10);
  }

  public void setLeftHeight(double newHeight) {
    //double precent = newHeight / 4;

    //double stageBottomY = 6 * precent;

    //leftBottomRoot.setPosition(leftRootX, stageBottomY);
  }

  public void setRightHeight(double newHeight) {
    //double precent = newHeight / 4;

    //double stageBottomY = 6 * precent;

    //rightBottomRoot.setPosition(leftRootX, stageBottomY);
  }
}
