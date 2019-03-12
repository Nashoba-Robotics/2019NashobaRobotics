package edu.nr.robotics.subsystems.lift;

import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
import edu.nr.robotics.subsystems.EnabledSubsystems;

public class LiftJoystickCommand extends JoystickCommand{

    private static final double MIN_LIFT_JOYSTICK_PERCENT = -0.1;
	private static double MAX_LIFT_JOYSTICK_PERCENT = 0.2;

    private static double motorPercent = 0;
    


    public LiftJoystickCommand() {
        super(Lift.getInstance());
    }

    protected void onExecute() {
        /*if (OI.getInstance().isKidModeOn()) {
			MAX_LIFT_JOYSTICK_PERCENT = 0.1;
		} else {
			MAX_LIFT_JOYSTICK_PERCENT = 0.4;
        }*/

        if(!OI.getInstance().isLiftNonZero()){
            if (Lift.getInstance().getPosition().lessThan(new Distance(3, Distance.Unit.INCH))) {
				Lift.getInstance().setMotorSpeedRaw(0);
            } else {
                Lift.getInstance().setMotorSpeedRaw(Lift.MIN_MOVE_VOLTAGE_PERCENT_LIFT);
            }
        }
        else if(EnabledSubsystems.LIFT_DUMB_ENABLED){
            motorPercent = OI.getInstance().getLiftJoystickValue();
            Lift.getInstance().setMotorSpeedRaw(motorPercent);
        } else if (OI.getInstance().getLiftJoystickValue() > 0){
            motorPercent = OI.getInstance().getLiftJoystickValue() * (MAX_LIFT_JOYSTICK_PERCENT - MIN_LIFT_JOYSTICK_PERCENT) + MIN_LIFT_JOYSTICK_PERCENT;
            Lift.getInstance().setMotorSpeedPercent(motorPercent);
        } else if (OI.getInstance().getLiftJoystickValue() < 0){
            motorPercent = OI.getInstance().getLiftJoystickValue() * (MAX_LIFT_JOYSTICK_PERCENT - MIN_LIFT_JOYSTICK_PERCENT) - MIN_LIFT_JOYSTICK_PERCENT;
            Lift.getInstance().setMotorSpeedPercent(motorPercent);
        }
    }

    protected boolean shouldSwitchToJoystick() {
        return Lift.getInstance().getCurrentCommand() == null || (!Robot.getInstance().isAutonomous() && OI.getInstance().isLiftNonZero());
    }

    protected long getPeriodOfCheckingForSwitchToJoystick() {
		return 100;
	}

}
