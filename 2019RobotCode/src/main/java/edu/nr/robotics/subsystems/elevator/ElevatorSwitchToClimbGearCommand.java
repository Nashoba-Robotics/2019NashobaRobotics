package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.elevator.Elevator.Gear;

public class ElevatorSwitchToClimbGearCommand extends NRCommand {

    public ElevatorSwitchToClimbGearCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        Elevator.getInstance().switchToClimbGear();
        
    }

    protected boolean isFinishedNR() {
        return true;
    }

}