package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorBottomCommand extends NRCommand {

    double percent;

    public ElevatorBottomCommand(double percent) {
        super(Elevator.getInstance());
        this.percent = percent;
    }

    protected void onStart() {
        Elevator.getInstance().setMotorPercentRaw(-Math.abs(percent));
    }

    protected void onEnd() {
        new ElevatorHoldClimbCommand().start();
    }

    protected boolean isFinishedNR() {
        return Elevator.getInstance().getPosition().lessThan(new Distance(1, Distance.Unit.INCH));
    }

}
