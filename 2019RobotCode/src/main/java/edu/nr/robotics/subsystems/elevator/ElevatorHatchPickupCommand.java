package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism;

public class ElevatorHatchPickupCommand extends NRCommand {
    private double holdPercent;

    public ElevatorHatchPickupCommand() {
        holdPercent = Elevator.HOLD_BOTTOM_PERCENT;
    }
    /*
    protected void onStart() {
        Elevator.holdingBottom = true;
        Elevator.getInstance().setMotorPercentRaw(holdPercent);
    }

    protected void onEnd() {
        Elevator.holdingBottom = false;
        Elevator.getInstance().setMotorPercentRaw(0);
    }

    protected boolean isFinishedNR() {
        return HatchMechanism.getInstance().hasHatch();
    }*/


}