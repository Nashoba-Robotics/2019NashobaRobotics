package edu.nr.lib.commandbased;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Every subsystem that extends NRSubsystem should have a singleton.
 * 
 * The static function to initialize the singleton should, after initializing the singleton, call setJoystickCommand().
 * 
 * initDefaultCommand() and setDefaultCommand() should never be called.
 */
public abstract class NRSubsystem extends Subsystem implements SmartDashboardSource, Periodic {

	public static ArrayList<NRSubsystem> subsystems = new ArrayList<>();

	public abstract void disable();
	
	JoystickCommand joystickCommand;
	
	Timer switchToJoystickTimer;
	
	public NRSubsystem() {
		NRSubsystem.subsystems.add(this);
		periodics.add(this);
		sources.add(this);
	}
	
	/**
	 * Set the joystick command to be used by the subsystem.
	 * @param joystickCommand
	 */
	public final void setJoystickCommand(JoystickCommand joystickCommand) {
		this.joystickCommand = joystickCommand;
		
		switchToJoystickTimer = new Timer();
		switchToJoystickTimer.schedule(new JoystickSwitchChecker(), 1000, joystickCommand.getPeriodOfCheckingForSwitchToJoystick());
		initDefaultCommand();
	}
	
	/**
	 * Chooses the default command when the class is initialized
	 */
	@Override
	protected final void initDefaultCommand() {
		setDefaultCommand(joystickCommand);
	}
	
	private final class JoystickSwitchChecker extends TimerTask {

		@Override
		public void run() {
			if(joystickCommand.shouldSwitchToJoystick()) {
				Command currentCommand = getCurrentCommand();
				if(currentCommand != joystickCommand) {
					System.err.println(joystickCommand + " is overriding.");
					NRCommand.cancelCommand(getCurrentCommand());
				}
			}
		}
		
	}

}
