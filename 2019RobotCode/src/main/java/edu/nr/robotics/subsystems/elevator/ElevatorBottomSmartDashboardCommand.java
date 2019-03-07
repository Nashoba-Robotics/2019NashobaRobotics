package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorBottomSmartDashboardCommand extends NRCommand {

    double percent;

    public ElevatorBottomSmartDashboardCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        percent = Elevator.PROFILE_VEL_PERCENT_ELEVATOR;
        Elevator.getInstance().setMotorPercentRaw(-Math.abs(percent));
    }

    protected void onEnd() {
        new ElevatorHoldClimbCommand().start();
    }

    protected boolean isFinishedNR() {
        return Elevator.getInstance().getPosition().lessThan(new Distance(1, Distance.Unit.INCH));
    }

}
