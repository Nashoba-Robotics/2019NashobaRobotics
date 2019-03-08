
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
        Lift.getInstance().setLiftOutputRange(-Lift.profilePercent,Lift.profilePercent);
        Lift.getInstance().setPosition(setPoint);
    }

    protected void onEnd() {
        Lift.getInstance().setLiftOutputRange(-1, 1);
    }

    protected boolean isFinishedNR() {
        return false;
    }

}
