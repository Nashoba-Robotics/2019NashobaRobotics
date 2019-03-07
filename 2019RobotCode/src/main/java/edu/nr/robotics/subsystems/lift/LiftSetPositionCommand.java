package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

/**
 * Sets the lift to a specified height
 */
public class LiftSetPositionCommand extends NRCommand {

    Distance posSetPoint;

    public LiftSetPositionCommand(Distance posSetPoint) {
        super(Lift.getInstance());
        this.posSetPoint = posSetPoint;

    }

    protected void onStart() {
        Lift.getInstance().setLiftOutputRange(-0.5, 0.5);

        Lift.getInstance().setPosition(posSetPoint);

        if(!posSetPoint.equals(Distance.ZERO)) {
            Lift.getInstance().deployed = true;
        } else {
            Lift.getInstance().deployed = false;
        }
    }

    protected void onEnd() {
        Lift.getInstance().setLiftOutputRange(-1, 1);
    }

    protected boolean isFinishedNR() {
        return false;
    }

}
