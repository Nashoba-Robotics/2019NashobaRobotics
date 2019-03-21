package edu.nr.robotics.subsystems.auxiliarydrive;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.lift.Lift;

public class AuxiliaryDriveJoystickCommand extends JoystickCommand {

    public AuxiliaryDriveJoystickCommand() {
        super(AuxiliaryDrive.getInstance());
    }

    @Override
    protected void onExecute() {
        double moveValue = OI.getInstance().getArcadeMoveValue();

        if (Lift.deployed)
            AuxiliaryDrive.getInstance().setMotorSpeedInPercent(moveValue);
        
    }

    @Override
    protected boolean shouldSwitchToJoystick() {
        return Lift.getInstance().deployed && AuxiliaryDrive.getInstance().getCurrentCommand() == null;
    }

    @Override
    protected long getPeriodOfCheckingForSwitchToJoystick() {
        return 100;
    }
}
