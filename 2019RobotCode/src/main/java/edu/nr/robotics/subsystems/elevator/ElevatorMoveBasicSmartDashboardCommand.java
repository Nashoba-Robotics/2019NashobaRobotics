package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorMoveBasicSmartDashboardCommand extends NRCommand {

    private Distance initialPos;

    public ElevatorMoveBasicSmartDashboardCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        initialPos = Elevator.getInstance().getPosition();
        Elevator.getInstance().setMotorSpeedPercent(Math.abs(Elevator.PROFILE_VEL_PERCENT_ELEVATOR)
         * Elevator.profilePos.signum());
    }

    protected boolean isFinishedNR() {
     
        return (Elevator.getInstance().getPosition().sub(initialPos)).abs().greaterThan((Elevator.profilePos).abs());
    }

}