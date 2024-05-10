package frc.robot.subsystems.Climb;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface ClimbIO {

    @AutoLog
    public static class ClimbInputs {
        public double averageCurrent = 0.0;
        public double leftPosition = 0.0;
        public double rightPosition = 0.0;
    }

    void updateInputs(final ClimbInputs inputs);

    /*** Stops Climb motors */
    Command StopClimb();

    /*** Sets voltage of climb motors 
     * @param leftVoltage 
     * @param rightVoltage 
     * @return Command construct
     */
    Command setVoltage(double leftVoltage, double rightVoltage);

    /*** Gets average current of climb motors * @return double */
    double getCurrent();

}
