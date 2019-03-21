package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

/**
 * Sets the lift to a specified height
 */
public class LiftSetPositionCommand extends NRCommand {

    Distance posSetPoint;
    double outputRange;

    public LiftSetPositionCommand(Distance posSetPoint, double outputRange) {
        super(Lift.getInstance());
        this.posSetPoint = posSetPoint;
        this.outputRange = outputRange;

    }

    protected void onStart() {
        Lift.getInstance().setLiftOutputRange(-outputRange, outputRange);

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
