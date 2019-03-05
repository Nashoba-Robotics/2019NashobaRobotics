package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.Timer;

public class HKickCommand extends NRCommand {
    double drivePercent = 0;
    Time kickTime = Time.ZERO;

    double initialTime = 0;
    double currentTime = 0;

    boolean finished = false;

    public HKickCommand(double drivePercent, Time kickTime) {
        super(Drive.getInstance());

        this.drivePercent = drivePercent;
        this.kickTime = kickTime;
    }

    public void onStart() {
        Drive.getInstance().setMotorSpeedInPercent(0, 0, drivePercent);
        initialTime = Timer.getFPGATimestamp();
    }

    public void onEnd() {
        Drive.getInstance().setMotorSpeedInPercent(0, 0, 0);
    }

    public boolean isFinishedNR() {
        System.out.println("stuff happening");
        currentTime = Timer.getFPGATimestamp();
        return (currentTime - initialTime) > kickTime.get(Time.Unit.SECOND);
    }
}