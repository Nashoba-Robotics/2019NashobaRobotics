package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorCurrentCommand extends NRCommand{

    double percent;
    double current;

    boolean finished;
    double time;
    double spiketime;

    public ElevatorCurrentCommand(double percent, double current) {
        this.percent = percent;
        this.current = current;
    }

    protected void onStart() {
        Elevator.getInstance().setMotorPercentRaw(percent);
    }

    protected void onEnd() {
        Elevator.getInstance().disable();
    }

    protected boolean isFinishedNR() {
        if (Elevator.getInstance().getMasterCurrent() > current) {
			spiketime = Timer.getFPGATimestamp();
			if ((spiketime - time) > 0.05) 
				finished = true;
		}
		else {
            time = Timer.getFPGATimestamp();
            finished = false;
		}

        return finished;
    }
}
