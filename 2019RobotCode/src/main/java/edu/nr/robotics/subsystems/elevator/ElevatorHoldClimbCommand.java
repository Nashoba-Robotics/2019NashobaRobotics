package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class ElevatorHoldClimbCommand extends NRCommand {

    public ElevatorHoldClimbCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        Elevator.getInstance().setMotorPercentRaw(Elevator.MIN_MOVE_VOLTAGE_PERCENT_CLIMB_UP);
    }

    protected boolean isFinishedNR() {
        return false;
    }
}
