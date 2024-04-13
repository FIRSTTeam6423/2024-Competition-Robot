package frc.robot.simWidgets;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimbWidget {

  private final Mechanism2d mech = new Mechanism2d(15, 45);
  private final MechanismRoot2d root = mech.getRoot("climb", 2, 0);

  private final MechanismLigament2d arm = root.append(new MechanismLigament2d("arm", 10, 90));

  public ClimbWidget() {
    SmartDashboard.putData("Widgets/Climb", mech);
  }

}
