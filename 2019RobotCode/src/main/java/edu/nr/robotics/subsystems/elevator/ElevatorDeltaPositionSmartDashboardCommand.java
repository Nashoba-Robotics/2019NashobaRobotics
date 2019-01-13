package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorDeltaPositionSmartDashboardCommand extends NRCommand {
    private Distance initialPos;

    public ElevatorDeltaPositionSmartDashboardCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        this.initialPos = Elevator.getInstance().getPosition();
		Elevator.getInstance().setPosition(initialPos.add(Elevator.profilePos));
    }

    protected boolean isFinishedNR() {
        boolean finished = Elevator.getInstance().getVelocity().lessThan(Elevator.PROFILE_STOP_SPEED_THRESHOLD)
        && (initialPos.add(Elevator.profilePos).sub(Elevator.getInstance().getPosition())).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);

        return finished;
    }

    





}

