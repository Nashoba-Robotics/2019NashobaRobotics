package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.OI;
import edu.nr.robotics.subsystems.intakerollers.IntakeRollers;

public class ToggleKidModeCommand extends NRCommand {

    public static boolean kidModeEnabled = false;

    @Override
    protected void onStart() {
        kidModeEnabled = !kidModeEnabled;

        if (kidModeEnabled) {
            OI.getInstance().setDriveSpeedMultiplier(0.5);
            OI.getInstance().setElevatorSpeedMultiplier(0.5);
            IntakeRollers.getInstance().HOLD_PERCENT = 0.15;
        } else {
            OI.getInstance().setDriveSpeedMultiplier(1);
            OI.getInstance().setElevatorSpeedMultiplier(1);
            IntakeRollers.getInstance().HOLD_PERCENT = 0.27;
        }
    }
}
