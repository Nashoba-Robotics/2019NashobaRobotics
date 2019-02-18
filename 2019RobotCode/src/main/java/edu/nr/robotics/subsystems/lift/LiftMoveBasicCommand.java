package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class LiftMoveBasicCommand extends NRCommand {

    double percent;
    Distance dist;
    Distance initialPos;

    public LiftMoveBasicCommand(double percent, Distance dist) {
        super(Lift.getInstance());
        this.percent = percent;
        this.dist = dist;
        this.initialPos = Lift.getInstance().getPosition();
    }

    protected void onStart() {
        Lift.getInstance().setMotorSpeedRaw(Math.abs(percent) * dist.signum());
    }

    protected void onEnd() {
        Lift.getInstance().setMotorSpeedRaw(0);
    }

    protected boolean isFinishedNR() {
        return (Lift.getInstance().getPosition().sub(initialPos.add(dist))).abs()
        .lessThan(Lift.PROFILE_END_THRESHOLD_LIFT);
    }

}
