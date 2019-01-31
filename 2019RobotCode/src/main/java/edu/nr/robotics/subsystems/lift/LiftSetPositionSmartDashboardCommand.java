
package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class LiftSetPositionSmartDashboardCommand extends NRCommand {

    Distance setPoint;

    public LiftSetPositionSmartDashboardCommand() {
        super(Lift.getInstance());
    }

    protected void onStart() {
        this.setPoint = Lift.setPos;

        Lift.getInstance().setPosition(setPoint);
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
