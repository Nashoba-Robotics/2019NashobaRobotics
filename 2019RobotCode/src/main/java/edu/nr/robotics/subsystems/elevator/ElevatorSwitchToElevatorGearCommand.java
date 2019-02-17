package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.elevator.Elevator.Gear;

public class ElevatorSwitchToElevatorGearCommand extends NRCommand {

    public ElevatorSwitchToElevatorGearCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        Elevator.getInstance().switchToElevGear();
        
    }

    protected boolean isFinishedNR() {
        return true;
    }

}