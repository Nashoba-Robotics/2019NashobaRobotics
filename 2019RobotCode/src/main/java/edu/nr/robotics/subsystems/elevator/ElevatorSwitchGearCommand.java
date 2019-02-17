package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.elevator.Elevator.Gear;

public class ElevatorSwitchGearCommand extends NRCommand {

    public ElevatorSwitchGearCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {

        if(Elevator.getInstance().getCurrentGear() == Gear.elevator) {
            Elevator.getInstance().switchToClimbGear();
        } else {
            Elevator.getInstance().switchToElevGear();
        }
    }

    protected boolean isFinishedNR() {
        return true;
    }

}