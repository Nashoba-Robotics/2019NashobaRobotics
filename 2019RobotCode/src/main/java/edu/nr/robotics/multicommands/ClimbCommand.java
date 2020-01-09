package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.auxiliarydrive.AuxiliaryDrive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.lift.Lift;
import edu.nr.robotics.subsystems.liftlockmechanism.LiftLockMechanism;

public class ClimbCommand extends NRCommand {

    Distance initElevPos;

    public ClimbCommand() {
        super(new NRSubsystem[] {Elevator.getInstance(), Lift.getInstance(), LiftLockMechanism.getInstance(),  AuxiliaryDrive.getInstance()});
    }

    protected void onStart() {
        initElevPos = Elevator.getInstance().getPosition();
        LiftLockMechanism.getInstance().deployLiftLockMechanism();
        Elevator.getInstance().switchToClimbGear();
        Elevator.getInstance().setMotorPercentRaw(-Math.abs(Elevator.CLIMB_PERCENT));
        AuxiliaryDrive.getInstance().setMotorSpeedInPercent(AuxiliaryDrive.HOLD_CLIMB_PERCENT);
    }

    protected void onExecute() {
        //Lift.getInstance().setMotorSpeed(Elevator.getInstance().getVelocity().mul(-1));
        Lift.getInstance().setPosition((Elevator.getInstance().getPosition().sub(initElevPos)).mul(-1).add(Lift.LIFT_LEAD_DISTANCE));
        //Lift.getInstance().setMotorSpeed(Elevator.getInstance().getVelocity().add(new Speed(Lift.getInstance().P_Angle * Pigeon.getPigeon(Drive.getInstance().getPigeonTalon()).getPitch().get(Angle.Unit.DEGREE), Distance.Unit.FOOT, Time.Unit.SECOND).mul(-1)));
    
    }

    protected void onEnd() {
        new HoldClimbCommand((Elevator.getInstance().getPosition().sub(initElevPos)).mul(-1).add(Lift.LIFT_LEAD_DISTANCE), initElevPos).start();
        AuxiliaryDrive.getInstance().disable();
    }

    protected boolean isFinishedNR() {
        return Elevator.getInstance().getPosition().lessThan(Elevator.CLIMB_END_DISTANCE);
        //Lift.getInstance().getPosition().sub(initElevPos).abs().lessThan(Lift.getInstance().PROFILE_END_THRESHOLD_LIFT)
        // && Lift.getInstance().getVelocity().lessThan(Lift.getInstance().PROFILE_STOP_SPEED_THRESHOLD);
    }

}