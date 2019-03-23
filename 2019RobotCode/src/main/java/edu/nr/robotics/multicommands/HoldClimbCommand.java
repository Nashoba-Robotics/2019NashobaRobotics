package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.lift.Lift;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class HoldClimbCommand extends NRCommand {

    Distance liftPos;

    public HoldClimbCommand(Distance liftPos) {
        super(new NRSubsystem [] {Lift.getInstance(), Elevator.getInstance()});
        this.liftPos = liftPos;
    }

    protected void onStart() {
        Lift.getInstance().setPosition(liftPos);
        Elevator.getInstance().setMotorPercentRaw(Elevator.MIN_MOVE_VOLTAGE_PERCENT_CLIMB_UP);
    }


    protected void onEnd() {
        new RetractLiftCommand().start();
    }

    protected boolean isFinishedNR() {
        return false;//!EnabledSensors.platformSensor.get();
    }

}