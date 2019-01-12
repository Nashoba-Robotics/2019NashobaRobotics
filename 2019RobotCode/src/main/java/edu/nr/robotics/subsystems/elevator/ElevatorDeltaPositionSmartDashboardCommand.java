package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorDeltaPositionSmartDashboardCommand extends NRCommand {
    private Distance initialPos;

    public ElevatorDeltaPositionSmartDashboardCommand() {
        super(Elevator.getInstance());
    }
}

