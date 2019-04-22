package edu.nr.robotics.subsystems.auxiliarydrive;

import edu.nr.lib.commandbased.NRCommand;

public class AuxiliaryDriveSetMotorPercentRawCommand extends NRCommand {

    double percent;

    public AuxiliaryDriveSetMotorPercentRawCommand(double percent) {
        super(AuxiliaryDrive.getInstance());
    }

    protected void onStart() {
        AuxiliaryDrive.getInstance().setMotorSpeedRaw(percent);
    }

    protected boolean isFinishedNR() {
        return true;
    }
}
