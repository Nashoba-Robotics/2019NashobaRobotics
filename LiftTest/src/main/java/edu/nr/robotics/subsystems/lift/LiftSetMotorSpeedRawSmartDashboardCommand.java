
package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;

public class LiftSetMotorSpeedRawSmartDashboardCommand extends NRCommand {

    double frontPercent;
    double backPercent;

    public LiftSetMotorSpeedRawSmartDashboardCommand() {
        super(Lift.getInstance());
    }

    protected void onStart() {
        this.frontPercent = Lift.frontPercent;
        this.backPercent = Lift.backPercent;
        Lift.getInstance().setMotorSpeedRaw(frontPercent, backPercent);
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
