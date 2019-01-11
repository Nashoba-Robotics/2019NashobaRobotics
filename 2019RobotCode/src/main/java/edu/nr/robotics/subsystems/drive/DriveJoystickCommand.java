package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.JoystickCommand;
import edu.nr.lib.gyro.GyroCorrection;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
import edu.nr.robotics.subsystems.drive.Drive.DriveMode;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.OI;


public class DriveJoystickCommand extends JoystickCommand {

    double prevTime = 0;

    private GyroCorrection gyroCorrection;

    public DriveJoystickCommand() {
        super(Drive.getInstance());
    }

    public void onStart() {
        gyroCorrection = new GyroCorrection();
    }

    public void onExecute() {
        double dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
        prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        switch (OI.drivemode) {

        }
    }

}

