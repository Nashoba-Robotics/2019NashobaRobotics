package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorHoldPositionCommand extends NRCommand {

    public ElevatorHoldPositionCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        Elevator.getInstance().positionPID(Elevator.getInstance().getPosition());
    }

    protected boolean isFinishedNR() {
        return false;
    }

}
