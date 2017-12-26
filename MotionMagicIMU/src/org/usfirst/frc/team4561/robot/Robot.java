/**
 * Example demonstrating the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the 
 * cause, flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * and followed the walk-through in the Talon SRX Software Reference Manual,
 * use button1 to motion-magic servo to target position specified by the gamepad stick.
 */
package org.usfirst.frc.team4561.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import com.ctre.CANTalon.*;
import edu.wpi.first.wpilibj.Timer;

// EAP SPI bus IMU
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

// EAP expansion port IMU
//import com.analog.adis16448.frc.ADIS16448_IMU;

    
        

public class Robot extends IterativeRobot {
	CANTalon _talon = new CANTalon(2);// EAP Changed to 2 from 3
	Joystick _joy = new Joystick(0);
	StringBuilder _sb = new StringBuilder();
	// EAP Expansion port IMU
	 //ADIS16448_IMU imu;
	// EAP SPI bus IMU
	 ADXRS450_Gyro imu;

	public void robotInit() {
		// EAP Add IMU
		// EAP expansion port IMU 
		//imu = new ADIS16448_IMU();
		// EAP SPI BUS IMU
		imu = new ADXRS450_Gyro();
		/* first choose the sensor */
		// EAP We need to set the feedback device
		_talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    _talon.reverseSensor(true);// EAP TBD May not be true in our case
		// EAP TBD Below needs to be uncommented and implemented
		 _talon.configEncoderCodesPerRev(2048); // if using FeedbackDevice.QuadEncoder
		// EAP We are not using POT so leave commented out
		// _talon.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputVoltage(+0.0f, -0.0f);
		_talon.configPeakOutputVoltage(+12.0f, -12.0f);
		/* set closed loop gains in slot0 - see documentation */
		//EAP usage setProfile- 
		// set which profile to set the pid constants for. 
		// You can have two profiles, with values of 0 or 1, 
		// allowing you to keep a second set of values on hand in the talon. 
		// In order to switch profiles without recalling setPID, you must call setProfile().
		_talon.setProfile(0);
		// EAP set PIDF values for profile 0
		// EAP these need tuning see 12.8. Motion Magic Closed-Loop Walkthrough – Java 
		// EAP of the talon SW ref manual.
		_talon.setF(0.017);
		_talon.setP(0);
		_talon.setI(0);
		_talon.setD(0);
		/* set acceleration and vcruise velocity - see documentation */
		//_talon.setMotionMagicCruiseVelocity(0);
		//_talon.setMotionMagicAcceleration(0);
		// EAP Both these state that value is in rpm not native units
		// EAP this needs checking.
		// EAP Set cruise velocity in rpm
		// EAP test station full throttle calculated to be 8724 RPM
		// Lets keep it low @ %25 or 2181
		_talon.setMotionMagicCruiseVelocity(2181);
		// EAP set accel in rpm
		// The RPM set here is 1 sec ramp. So if we set max
	    // RPM the wheel should take 1 sec to get to max speed
		// If we set 1/2 max it will take 2 sec etc.
	    _talon.setMotionMagicAcceleration(2181/2);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getAxis(AxisType.kY);
		/* calculate the percent motor output */
		double motorOutput = _talon.getOutputVoltage() / _talon.getBusVoltage();
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(motorOutput);
		_sb.append("\tspd:");
		_sb.append(_talon.getSpeed());
		// EAP Send IMU data the SD 
		SmartDashboard.putData("IMU", imu);
		SmartDashboard.putDouble("IMU Angle", imu.getAngle());


		if (_joy.getRawButton(1)) {
			/* Motion Magic */
			double targetPos = leftYstick
					* 10.0; /* 10 Rotations in either direction */
			_talon.changeControlMode(TalonControlMode.MotionMagic);
			_talon.set(targetPos); 

			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError());
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} 
	    else {
		/* Percent voltage mode */
		_talon.changeControlMode(TalonControlMode.PercentVbus);
		_talon.set(0); // EAP for now no movement when not in MM
        // _talon.set(leftYstick);
		}
		/* instrumentation */
		// EAP sends talon info to smartdashboard and print _sb to system.out
		Instrum.Process(_talon, _sb);
	}
}