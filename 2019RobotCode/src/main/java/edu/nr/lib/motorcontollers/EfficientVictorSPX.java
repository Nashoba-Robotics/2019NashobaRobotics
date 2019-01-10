
package edu.nr.lib.motorcontollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class EfficientVictorSPX extends VictorSPX {

    protected double mLastSet = 0;
    protected ControlMode mLastControlMode = ControlMode.PercentOutput;

    /**
     * Creates new EfficientVictorSPX
     * @param deviceNumber
     */
    public EfficientVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }
    
    @Override
    public void set(ControlMode controlMode, double value) {
    	if (value != mLastSet || controlMode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = controlMode;
            super.set(mLastControlMode, mLastSet);
    	}
    }

}
