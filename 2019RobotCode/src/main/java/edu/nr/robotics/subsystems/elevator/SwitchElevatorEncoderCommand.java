package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;

public class SwitchElevatorEncoderCommand extends NRCommand {

    public SwitchElevatorEncoderCommand() {
        super(Elevator.getInstance());
    }

    protected void onStart() {
        Elevator.getInstance().setMotorPercentRaw(0);
        //Elevator.getInstance().switchEncoderTalon();
    }

    protected boolean isFinishedNR() {
        return true;
    }
}
