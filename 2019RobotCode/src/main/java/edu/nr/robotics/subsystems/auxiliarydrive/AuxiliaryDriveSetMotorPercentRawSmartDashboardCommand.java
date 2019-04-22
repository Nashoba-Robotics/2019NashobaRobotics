package edu.nr.robotics.subsystems.auxiliarydrive;

import edu.nr.lib.commandbased.NRCommand;

public class AuxiliaryDriveSetMotorPercentRawSmartDashboardCommand extends NRCommand {

    public AuxiliaryDriveSetMotorPercentRawSmartDashboardCommand() {
        super(AuxiliaryDrive.getInstance());
    }

    protected void onStart() {
        AuxiliaryDrive.getInstance().setMotorSpeedRaw(AuxiliaryDrive.DRIVE_PERCENT);
    }
}
