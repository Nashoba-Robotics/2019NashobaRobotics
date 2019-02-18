package edu.nr.robotics;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.GearSwitcher.Gear;

public class SwitchGearCommand extends NRCommand {

	Gear gear;
	
	public SwitchGearCommand() {
        super(GearSwitcher.getInstance());
    }
    
	@Override
	public void onStart() {
        this.gear = GearSwitcher.getInstance().getCurrentGear();
        
		if(gear == Gear.low) {
			GearSwitcher.getInstance().switchToHighGear();
		} else {
			GearSwitcher.getInstance().switchToLowGear();
		}
    }
    
    @Override
    public boolean isFinishedNR() {
        return true;
    }
	
}