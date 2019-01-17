
package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class LiftSetPositionSmartDashboardCommand extends NRCommand {

    Distance frontSetPoint;
    Distance backSetPoint;

    public LiftSetPositionSmartDashboardCommand() {
        super(Lift.getInstance());
    }

    protected void onStart() {
        this.frontSetPoint = Lift.frontSetPos;
        this.backSetPoint = Lift.backSetPos;

        Lift.getInstance().setPosition(frontSetPoint, backSetPoint);
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
