package edu.nr.robotics.subsystems.hatchmechanism;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.subsystems.hatchmechanism.HatchMechanism.State;

public class ScoreHatchCommand extends NRCommand {

    public ScoreHatchCommand() {
        super(HatchMechanism.getInstance());
    }

    protected void onStart() {
        if(HatchMechanism.getInstance().hasHatch()) {
            if(HatchMechanism.getInstance().currentHatchState() == State.RETRACTED) {
                HatchMechanism.getInstance().deployHatchMechanism();
            }
            
            HatchMechanism.getInstance().releaseHatch();
            HatchMechanism.getInstance().retractHatchMechanism();
        }
    
    }

    protected boolean isFInishedNR() {
        return true;
    }

}