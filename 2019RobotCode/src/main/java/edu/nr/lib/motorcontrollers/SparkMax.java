package edu.nr.lib.motorcontrollers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMax {

    private static final int TIMEOUT = 0;
    private static final IdleMode MODE = IdleMode.kBrake; 
    private static final boolean INVERT = false;
    private static final double RAMP_RATE = 0;
    private static final int SMART_LIMIT = 40;
    private static final int SECONDARY_LIMIT = 60;
    
    public static CANSparkMax createSpark(int deviceNumber, MotorType type) {
        CANSparkMax spark = new CANSparkMax(deviceNumber, type);
        spark.setCANTimeout(TIMEOUT);
        spark.setIdleMode(MODE);
        spark.setInverted(INVERT);
        spark.setClosedLoopRampRate(RAMP_RATE);
        spark.setOpenLoopRampRate(RAMP_RATE);

        if (type == MotorType.kBrushless)
            spark.setSmartCurrentLimit(SMART_LIMIT);

        spark.setSecondaryCurrentLimit(SECONDARY_LIMIT);
        return spark;
    }

    public static CANSparkMax createSpark(int deviceNumber, boolean brushless) {
        if (brushless)
            return createSpark(deviceNumber, MotorType.kBrushless);
        else
            return createSpark(deviceNumber, MotorType.kBrushed);
    }

}
