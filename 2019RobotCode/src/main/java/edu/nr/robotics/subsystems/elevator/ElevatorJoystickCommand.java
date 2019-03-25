

package edu.nr.robotics.subsystems.elevator;


import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class ElevatorJoystickCommand extends JoystickCommand {
	
	private static final double MIN_ELEV_JOYSTICK_PERCENT = 0;
	private static double MAX_ELEV_JOYSTICK_PERCENT_UP = 0;
    private static double MAX_ELEV_JOYSTICK_PERCENT_DOWN = 0;
    private static double motorPercent = 0;
    
    public ElevatorJoystickCommand() {
        super(Elevator.getInstance());
    }

    protected void onExecute() {

        if (OI.getInstance().isKidModeOn()) {
			MAX_ELEV_JOYSTICK_PERCENT_UP = 0.3;
			MAX_ELEV_JOYSTICK_PERCENT_DOWN = 0.2;
		} else {
			MAX_ELEV_JOYSTICK_PERCENT_UP = 0.42;
			MAX_ELEV_JOYSTICK_PERCENT_DOWN = 0.3; // change for this year...
        }
        
        if(!OI.getInstance().isElevatorNonZero()) {

            if (Elevator.getInstance().getPosition().lessThan(new Distance(4, Distance.Unit.INCH))) {
				Elevator.getInstance().setMotorPercentRaw(0);
            } else if (Elevator.getInstance().getCurrentGear() == Elevator.Gear.elevator) { //if (!EnabledSensors.elevatorSensor.get())
                //new ElevatorHoldPositionCommand().start();
                Elevator.getInstance().setMotorPercentRaw(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP);
                //System.out.println("38");
        //    } else {
        //        Elevator.getInstance().setMotorPercentRaw(Elevator.REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN);
            }

        } else if (EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
            motorPercent = OI.getInstance().getElevatorJoystickValue();
            Elevator.getInstance().setMotorPercentRaw(motorPercent * MAX_ELEV_JOYSTICK_PERCENT_UP);
      
        } else if (OI.getInstance().getElevatorJoystickValue() > 0) {
			motorPercent = OI.getInstance().getElevatorJoystickValue() * (MAX_ELEV_JOYSTICK_PERCENT_UP - MIN_ELEV_JOYSTICK_PERCENT) + MIN_ELEV_JOYSTICK_PERCENT;
            Elevator.getInstance().setMotorSpeedPercent(motorPercent);
           // System.out.println("50" + motorPercent);
            
		} else if (OI.getInstance().getElevatorJoystickValue() < 0) {
			motorPercent = OI.getInstance().getElevatorJoystickValue() * (MAX_ELEV_JOYSTICK_PERCENT_DOWN - MIN_ELEV_JOYSTICK_PERCENT) - MIN_ELEV_JOYSTICK_PERCENT;
            Elevator.getInstance().setMotorSpeedPercent(motorPercent);
            //System.out.println("55");
        }
    }

    protected boolean shouldSwitchToJoystick() {
		return Elevator.getInstance().getCurrentCommand() == null || (/*!Robot.getInstance().isAutonomous() &&*/ OI.getInstance().isElevatorNonZero());
	}

	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}

