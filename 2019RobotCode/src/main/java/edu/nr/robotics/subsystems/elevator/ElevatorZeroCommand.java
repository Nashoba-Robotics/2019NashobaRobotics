package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorZeroCommand extends NRCommand {

    public ElevatorZeroCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        Elevator.getInstance().zeroElevEncoder();
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
