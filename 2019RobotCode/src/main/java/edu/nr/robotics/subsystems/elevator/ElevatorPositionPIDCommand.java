package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorPositionPIDCommand extends NRCommand {

    Distance pos;

    public ElevatorPositionPIDCommand(Distance pos) {
        super(Elevator.getInstance());
        this.pos = pos;
    }

    protected void onStart() {
        Elevator.getInstance().positionPID(pos);
    }

    protected boolean isFinishedNR() {
        return false;
    }

}
