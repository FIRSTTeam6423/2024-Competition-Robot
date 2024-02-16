package frc.robot.util;

public enum CargoState {
    IDLE, // amp mech is in stow
    INTAKING, // intake is down, rollers moving
    STOW, // intake up, note inside
    SPINUP, // everything stowed excet shoot motors
    SHOOT, // intake motors in reverse to push note into handoff & flywheels are at speed
    HANDOFF, // intake movers moving out to push note to shooters, shooter moters moving slowly to push into handoff
    DEPOSIT // amp mech takes note and moves it down
}
