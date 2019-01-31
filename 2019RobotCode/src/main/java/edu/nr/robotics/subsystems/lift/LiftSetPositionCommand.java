package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

/**
 * Sets the lift to a specified height
 */
public class LiftSetPositionCommand extends NRCommand {

    Distance posSetPoint;

    public LiftSetPositionCommand(Distance posSetPoint, Distance backSetPoint) {
        super(Lift.getInstance());
        this.posSetPoint = posSetPoint;

    }

    protected void onStart() {
        Lift.getInstance().setPosition(posSetPoint);
    }

    protected void onEnd() {
        if(!posSetPoint.equals(Distance.ZERO)) {
            Lift.getInstance().deployed = true;
        } else {
            Lift.getInstance().deployed = false;
        }
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
