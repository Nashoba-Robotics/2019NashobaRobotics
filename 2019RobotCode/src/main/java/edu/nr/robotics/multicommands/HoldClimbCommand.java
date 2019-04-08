package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.auxiliarydrive.AuxiliaryDrive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.lift.Lift;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Timer;

public class HoldClimbCommand extends NRCommand {

    Distance liftPos;
    Distance initClimbElevPos;

    double time;
    double startTime;

    public HoldClimbCommand(Distance liftPos, Distance initClimbElevPos) {
        super(new NRSubsystem [] {Lift.getInstance(), Elevator.getInstance(), AuxiliaryDrive.getInstance()});
        this.liftPos = liftPos;
        this.initClimbElevPos = initClimbElevPos;
    }

    protected void onStart() {
        Lift.getInstance().setPosition(liftPos);
        Elevator.getInstance().setMotorPercentRaw(Elevator.MIN_MOVE_VOLTAGE_PERCENT_CLIMB_UP);
        startTime = Timer.getFPGATimestamp();
    }

    protected void onExecute() {
        time = Timer.getFPGATimestamp();

        if (time - startTime > AuxiliaryDrive.CLIMB_DELAY.get(Time.Unit.SECOND)) {
            AuxiliaryDrive.getInstance().setMotorSpeedInPercent(AuxiliaryDrive.DRIVE_PERCENT);
        }

    }

    protected void onEnd() {
        AuxiliaryDrive.getInstance().disable();
        new RetractLiftCommand().start();
    }

    protected boolean isFinishedNR() {
        if (initClimbElevPos.lessThan(Elevator.CLIMB_LOW_HEIGHT_ELEVATOR.add(new Distance(3, Distance.Unit.INCH)))) {
            return false;
        }
        return EnabledSensors.platformSensor.get();
    }

}