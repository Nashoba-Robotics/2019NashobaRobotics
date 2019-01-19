
package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Speed;

public class LiftSetVelocitySmartDasboardCommand extends NRCommand {

    Speed frontSetVel;
    Speed backSetVel;

    public LiftSetVelocitySmartDasboardCommand() {
        super(Lift.getInstance());
    }

    protected void onStart() {
        this.frontSetVel = Lift.getInstance().frontSetVel;
        this.backSetVel = Lift.getInstance().backSetVel;
        Lift.getInstance().setMotorSpeed(frontSetVel, backSetVel);
    }

    protected boolean isFinishedNR() {
        return true;
    }

}
