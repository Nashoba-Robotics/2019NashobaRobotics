package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

/**
 * Sets the lift to a specified height
 */
public class LiftSetPositionCommand extends NRCommand {

    Distance frontSetPoint;
    Distance backSetPoint;

    public LiftSetPositionCommand(Distance frontSetPoint, Distance backSetPoint) {
        super(Lift.getInstance());
        this.frontSetPoint = frontSetPoint;
        this.backSetPoint = backSetPoint;

    }

    protected void onStart() {
        Lift.getInstance().setPosition(frontSetPoint, backSetPoint);
    }

    protected void onEnd() {
        if(!frontSetPoint.equals(Distance.ZERO) && !backSetPoint.equals(Distance.ZERO)) {
            Lift.getInstance().deployed = true;
        } else {
            Lift.getInstance().deployed = false;
        }
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
