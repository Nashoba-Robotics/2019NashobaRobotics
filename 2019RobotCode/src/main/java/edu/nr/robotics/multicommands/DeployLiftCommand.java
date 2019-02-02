package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.lift.Lift;

public class DeployLiftCommand extends NRCommand {

    Distance elevSetPoint;
    Distance liftSetPoint;

    public DeployLiftCommand(Distance elevSetPoint, Distance liftSetPoint) {
        super(new NRSubsystem[] {Elevator.getInstance(), Lift.getInstance()});
        this.elevSetPoint = elevSetPoint;
        this.liftSetPoint = liftSetPoint;
    }

    public DeployLiftCommand() {
        super(new NRSubsystem[] {Elevator.getInstance(), Lift.getInstance()});
        this.elevSetPoint = Elevator.getInstance().profilePos;
        this.liftSetPoint = Lift.getInstance().setPos;
    }

    protected void onStart() {
        Elevator.getInstance().switchToClimbGear();
        Elevator.getInstance().setPosition(elevSetPoint);
    }

    protected void onExecute(){
        Lift.getInstance().setMotorSpeed(Elevator.getInstance().getVelocity());
    }

    protected void onEnd() {
        if(!elevSetPoint.equals(Distance.ZERO) && !liftSetPoint.equals(Distance.ZERO)) {
            Lift.getInstance().deployed = true;
        } else {
            Lift.getInstance().deployed = false;
        }

        Lift.getInstance().setPosition(liftSetPoint);
    }

    protected boolean isFinishNR() {
        return Elevator.getInstance().getPosition().sub(elevSetPoint).abs().lessThan(Elevator.getInstance().PROFILE_END_POS_THRESHOLD_ELEVATOR)
         && Lift.getInstance().getPosition().sub(liftSetPoint).abs().lessThan(Lift.getInstance().PROFILE_END_THRESHOLD_LIFT)
         && Elevator.getInstance().getVelocity().lessThan(Elevator.getInstance().PROFILE_STOP_SPEED_THRESHOLD)
         && Lift.getInstance().getVelocity().lessThan(Lift.getInstance().PROFILE_STOP_SPEED_THRESHOLD);
    }

}
