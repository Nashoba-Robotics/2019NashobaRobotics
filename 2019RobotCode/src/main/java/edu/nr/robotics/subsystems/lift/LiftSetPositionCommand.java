package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

/**
 * Sets the lift to a specified height
 */
public class LiftSetPositionCommand extends NRCommand {

    Distance posSetPoint;
    double percent;

    public LiftSetPositionCommand(Distance posSetPoint, double percent) {
        super(Lift.getInstance());
        this.posSetPoint = posSetPoint;
        this.percent = percent;

    }

    protected void onStart() {
        Lift.getInstance().setLiftOutputRange(-percent, percent);
        Lift.getInstance().setPosition(posSetPoint);
    }

    protected void onEnd() {
        Lift.getInstance().setLiftOutputRange(-1, 1);

        if(!posSetPoint.equals(Distance.ZERO)) {
            Lift.getInstance().deployed = true;
        } else {
            Lift.getInstance().deployed = false;
        }
    }

    protected boolean isFinishedNR() {
        return false;
    }

}
