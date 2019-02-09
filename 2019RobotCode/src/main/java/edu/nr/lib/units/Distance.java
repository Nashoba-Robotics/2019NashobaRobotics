package edu.nr.lib.units;

import edu.nr.lib.Units;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.lift.Lift;


public class Distance {
	
	public static final Distance ZERO = new Distance(0, Unit.defaultUnit);
	private double val;
	private Unit type;
	
	public enum Unit implements GenericUnit {
		FOOT, INCH, METER, ENCODER_REV_LIFT, MAGNETIC_ENCODER_TICK_DRIVE, ENCODER_REV_H, MAGNETIC_ENCODER_TICK_ELEV, 
		MAGNETIC_ENCODER_TICK_AUX_DRIVE, MAGNETIC_ENCODER_TICK_CLIMBER;
		
		public static final Unit defaultUnit = INCH;
		
		/**
		 * For the drive
		 */
		private static final double ENCODER_TICK_DRIVE_PER_INCH = Drive.EFFECTIVE_ENC_TICK_PER_INCH_DRIVE;
		
		/**
		 * For the H drive
		 */
		private static final double ENCODER_REV_DRIVE_H_PER_INCH = Drive.EFFECTIVE_ENC_REV_PER_INCH_H_DRIVE;
		
		/**
		 * For the elevator
		 */
		private static final double ENCODER_TICK_ELEV_PER_INCH = Elevator.ENC_TICK_PER_INCH_CARRIAGE;

		/**
		 * For the lift
		 */
		private static final double INCH_PER_REVOLUTION_LIFT = Lift.INCH_PER_REVOLUTION_LIFT;

		private static final double FOOT_PER_INCH = 1.0/Units.INCHES_PER_FOOT;
		private static final double METER_PER_INCH = 1.0/Units.INCHES_PER_METER;
		
		public double convertToDefault(double val) {
			if(this == Unit.defaultUnit) {
				return val;
			}
			if(this == Unit.FOOT) {
				return val / FOOT_PER_INCH;
			}
			if(this == Unit.METER) {
				return val / METER_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_DRIVE) {
				return val / ENCODER_TICK_DRIVE_PER_INCH;
			}
			if(this == Unit.ENCODER_REV_H) {
				return val * ENCODER_REV_DRIVE_H_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_ELEV) {
				return val / ENCODER_TICK_ELEV_PER_INCH;
			}
			if(this == Unit.ENCODER_REV_LIFT) {
				return val * INCH_PER_REVOLUTION_LIFT;
			}
			return 0;
		}
		
		public double convertFromDefault(double val) {
			if(this == Unit.defaultUnit) {
				return val;
			}
			if(this == Unit.FOOT) {
				return FOOT_PER_INCH * val;
			}
			if(this == Unit.METER) {
				return METER_PER_INCH * val;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_DRIVE) {
				return ENCODER_TICK_DRIVE_PER_INCH * val;
			}
			if(this == Unit.ENCODER_REV_H) {
				return val / ENCODER_REV_DRIVE_H_PER_INCH;
			}
			if(this == Unit.MAGNETIC_ENCODER_TICK_ELEV) {
				return ENCODER_TICK_ELEV_PER_INCH * val;
			}
			if(this == Unit.ENCODER_REV_LIFT) {
				return val / INCH_PER_REVOLUTION_LIFT;
			}
			return 0;
		}
}
	
	public Distance(double val, Unit type) {
		this.val = val;
		this.type = type;
	}
	
	public double get(Unit toType) {
		return type.convert(val, toType);
	}

	public double getDefault() {
		return get(Unit.defaultUnit);
	}
	
	public Distance sub(Distance distanceTwo) {
		return new Distance(this.get(Unit.defaultUnit) - distanceTwo.get(Unit.defaultUnit), Unit.defaultUnit);
	}
	
	public Distance add(Distance distanceTwo) {
		return new Distance(this.get(Unit.defaultUnit) + distanceTwo.get(Unit.defaultUnit), Unit.defaultUnit);
	}
	
	public Distance mul(double x) {
		return new Distance(this.get(Unit.defaultUnit) * x, Unit.defaultUnit);
	}
	
	public boolean lessThan(Distance distanceTwo) {
		return this.get(Unit.defaultUnit) < distanceTwo.get(Unit.defaultUnit);
	}

	public boolean greaterThan(Distance distanceTwo) {
		return this.get(Unit.defaultUnit) > distanceTwo.get(Unit.defaultUnit);
	}
	
	public Distance negate() {
		return new Distance(-this.get(Unit.defaultUnit), Unit.defaultUnit);
	}
	
	public Distance abs() {
		return new Distance(Math.abs(this.get(Unit.defaultUnit)), Unit.defaultUnit);
	}
	
	public double signum() {
		return Math.signum(this.get(Unit.defaultUnit));
	}
	
	@Override
	public boolean equals(Object distanceTwo) {
		if(distanceTwo instanceof Distance) {
			return this.get(Unit.defaultUnit) == ((Distance) distanceTwo).get(Unit.defaultUnit);
		} else {
			return false;
		}
	}

	public double div(Distance distance) {
		return this.get(Unit.defaultUnit) / distance.get(Unit.defaultUnit);
	}

}
