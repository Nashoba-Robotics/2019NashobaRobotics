package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.drive.Drive;
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

    protected void onStart() {
        Elevator.getInstance().switchToClimbGear();
        Elevator.getInstance().setPosition(elevSetPoint);
    }

    protected void onExecute() {
        Lift.getInstance().setMotorSpeed(Elevator.getInstance().getVelocity().add(new Speed(Lift.getInstance().P_Angle * Pigeon.getPigeon(RobotMap.PIGEON_ID).getPitch().get(Angle.Unit.DEGREE), Distance.Unit.FOOT, Time.Unit.SECOND)));
    }

    protected void onEnd() {
        if(!elevSetPoint.equals(Distance.ZERO) && !liftSetPoint.equals(Distance.ZERO)) {
            Lift.getInstance().deployed = true;
        } else {
            Lift.getInstance().deployed = false;
        }

        Lift.getInstance().setPosition(liftSetPoint);
    }

    protected boolean isFinishedNR() {
        return Elevator.getInstance().getPosition().sub(elevSetPoint).abs().lessThan(Elevator.getInstance().PROFILE_END_POS_THRESHOLD_ELEVATOR)
         && Lift.getInstance().getPosition().sub(liftSetPoint).abs().lessThan(Lift.getInstance().PROFILE_END_THRESHOLD_LIFT)
         && Elevator.getInstance().getVelocity().lessThan(Elevator.getInstance().PROFILE_STOP_SPEED_THRESHOLD)
         && Lift.getInstance().getVelocity().lessThan(Lift.getInstance().PROFILE_STOP_SPEED_THRESHOLD);
    }

}
