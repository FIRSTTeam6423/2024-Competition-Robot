package frc.robot.subsystems.Climb;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface ClimbIO {

    @AutoLog
    public static class ClimbInputs {
        public double averageCurrent = 0.0;
        public double leftRotations = 0.0;
        public double rightRotations = 0.0;
        public double appliedVoltageLeft = 0.0;
        public double appliedVoltageRight = 0.0;
    }

    void updateInputs(final ClimbInputs inputs);

    /*** Stops Climb motors */
    Command stopClimb();

    /*** Sets voltage of climb motors 
     * @param leftVoltage 
     * @param rightVoltage 
     * @return Command construct
     */
    Command setVoltage(double leftVoltage, double rightVoltage);

    /*** Gets average current of climb motors * @return double */
    double getCurrent();

}
