package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class LiftMoveBasicSmartDashboardCommand extends NRCommand {

    double percent;
    Distance dist;
    Distance initialPos;

    public LiftMoveBasicSmartDashboardCommand() {
        super(Lift.getInstance());
    }

    protected void onStart() {
        this.percent = Lift.getInstance().profilePercent;
        this.dist = Lift.deltaPos;
        this.initialPos = Lift.getInstance().getPosition();
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
