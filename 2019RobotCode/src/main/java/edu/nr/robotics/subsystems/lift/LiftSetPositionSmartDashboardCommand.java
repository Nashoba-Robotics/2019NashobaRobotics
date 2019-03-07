
package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class LiftSetPositionSmartDashboardCommand extends NRCommand {

    Distance setPoint;

    public LiftSetPositionSmartDashboardCommand() {
        super(Lift.getInstance());
    }

    protected void onStart() {
        Lift.getInstance().setLiftOutputRange(-0.5, 0.5);

        this.setPoint = Lift.setPos;
        Lift.getInstance().setPosition(setPoint);

        if(!setPoint.equals(Distance.ZERO)) {
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
