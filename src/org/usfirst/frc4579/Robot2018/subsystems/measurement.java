// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4579.Robot2018.subsystems;

import org.usfirst.frc4579.Robot2018.RobotMap;
import org.usfirst.frc4579.Robot2018.commands.*;
import org.usfirst.frc4579.instrumentation.DebugTextFile;
import org.usfirst.frc4579.instrumentation.Instrumentation;

import com.eagles.sensors.MPU6050_I2C;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import flowSensor.FlowMotion;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class measurement extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	//------------------------------ Flow Motion Methods --------------------------//
    // Makes an instance of the flowMotion
	FlowMotion opticSensor = new FlowMotion();
	
	public boolean initFlowMotion(){
		return opticSensor.init();
	}
	
	public void getCounts(){
		opticSensor.getCounts();
	}

	public int getFlowMotionX(){
		return opticSensor.accumDeltaX;
	}
	
	public int getFlowMotionY(){
		return opticSensor.accumDeltaY;
	}
	
	public void resetFlowMotion(){
		opticSensor.reset();
	}
	
	public int getX(){
		return opticSensor.deltaX;
	}
	public int getY(){
		return opticSensor.deltaY;
	}
	
	
	//------------------------ Gyro Methods ------------------------------//
	private static final double MMtoInches = 0.0393701;      // Millimeters to inches
	
    private final MPU6050_I2C mpu   = new MPU6050_I2C(MPU6050_I2C.ACCELFULLSCALE.ACCEL2G, 
			  MPU6050_I2C.GYROFULLSCALE.DEGSEC250);
	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	private final double angleToAccelScale = 1.0;
	
	private double fieldPosX        = 0.0; // X distance in field frame.  Field origin is location of robot at beginning of match.
	private double fieldPosY        = 0.0; // Y distance in field frame.
	private double distanceX        = 0.0; // Robot frame.
	private double distanceY        = 0.0; // Robot frame.
	private double linealDistance   = 0.0;
	
	private double velocityX        = 0.0; // Robot frame.
	private double velocityY        = 0.0; // Robot frame.
	
	private double robotAngleRateZ  = 0.0;
	private double robotAngleZ      = 0.0;
	private double robotAngleY      = 0.0;
	private double robotAngleX      = 0.0;
	
	private double correctedRange   = 0.0;  // Range finder range.
	
	private double lastTime  = 0.0;
	
	private boolean firstCall       = true;
	private boolean mpuAvailable    = false;
	private boolean lidarAvailable  = false;
	private boolean lidarContinuous = true;  // True  => lidar makes continous back-to-back measurements.
		 // False => use lidar in single-shot mode.
	
	// Create a debug file containing data for this class for post-run
	// analysis.
	DebugTextFile measData = new DebugTextFile(
												"measData", 															// Base file name.
												true, 																	// Add time stamp to data.
												"AccelX\tAccelY\tAccelZ\tVelX\tVelY\tdistX\tdistY\tzAngleRate\tzAngle", // File header.
												20000);																	// Max anticipated lines in file.
	
	// Initializes the accelerometer and distance ranging devices.
	public void initialize() {
	
	mpuAvailable = mpu.init();
	
	
	if (mpuAvailable) {
	
		System.out.println("***** MEASUREMENT INITIALIZED" + "\n");
	
	}
	else {
		System.out.println("***** MPU INIT FAILED *********" + "\n");
		measData.write("MPU INIT FAILED");
	}
	
		reset();
	
		SmartDashboard.putBoolean ("MPU Available:", mpuAvailable);
	
	}
	
	
	// Resets the robot's current kinematic data.  All new kinematic computations
	// will be relative to the position of the robot at the time of this call.
	public void reset() {
		velocityX        = 0.0;
		velocityY        = 0.0;
		distanceX        = 0.0;
		distanceY        = 0.0;
		robotAngleZ      = 0.0;  
		robotAngleRateZ  = 0.0;
		robotAngleZ      = 0.0;
		robotAngleY      = 0.0;
		robotAngleX      = 0.0;
	}
	
	
	// Returns true if the accelerometer is working.
	public boolean positionAvailable() {
		return mpuAvailable;
	}
	
	// Compute accelerometer and range data.
	public void measure() {
	System.out.println(mpuAvailable);
		if (mpuAvailable) {
		
		// Compute change in time since last computations.
		double time   = Instrumentation.timeNow();
		if (firstCall) {
			lastTime = time;
			firstCall = false;
		}
		double deltaT = time - lastTime;
		System.out.println(deltaT);
		
		lastTime = time;

		
		/*
		// Compute new distance vector.
		double dX = velocityX * deltaT;
		distanceX += dX;
		double dY = velocityY * deltaT;
		distanceY += dY;
		
		linealDistance += Math.sqrt((dX * dX) + (dY * dY));
		*/
		
		// Compute new angular data.
		robotAngleRateZ = mpu.getGyroRateZ();
		robotAngleZ    += robotAngleRateZ * deltaT;
		System.out.println(robotAngleRateZ);
		//double angleRateY = mpu.getGyroRateY();
		//robotAngleY    += angleRateY * deltaT;
		//double angleRateX = mpu.getGyroRateX();
		//robotAngleX    += angleRateX * deltaT;
		
		/*
		// Compute the relative motion to absolute field coordinates.
		fieldPosX += dX * Math.sin(Math.toRadians(robotAngleZ));
		fieldPosY += dY * Math.sin(Math.toRadians(robotAngleZ));
		
		// Display the data.
		SmartDashboard.putString ("Field Pos X:", String.format("%7.1f", distanceX));
		SmartDashboard.putString ("Field Pos Y:", String.format("%7.1f", distanceY));
		SmartDashboard.putString ("Distance X:" , String.format("%7.1f", distanceX));
		SmartDashboard.putString ("Distance Y:" , String.format("%7.1f", distanceY));
		SmartDashboard.putString ("Velocity X:" , String.format("%7.2f", velocityX));
		SmartDashboard.putString ("Velocity Y:" , String.format("%7.2f", velocityY));
		*/
		SmartDashboard.putString ("Angle Z:"    , String.format("%7.1f", robotAngleZ));
		SmartDashboard.putString ("Angle Y:"    , String.format("%7.1f", robotAngleY));
		SmartDashboard.putString ("Angle X:"    , String.format("%7.1f", robotAngleX));
		
		measData.write(robotAngleRateZ + "\t" + robotAngleZ);
		
	}
	
	/*    	if (lidarAvailable) {
	
	double range;
	
	if (lidarContinuous) 
	range = lidar.readRangeContinuousMillimeters();
	else
	range = (double)lidar.readRangeSingleMillimeters();
	range *= MMtoInches;
	correctedRange = 1.125 * range;
	SmartDashboard.putString ("VL53LOX Rng:", String.format("%7.1f", range));
	SmartDashboard.putString ("VL53LOX Corrected Rng:", String.format("%7.1f", correctedRange));
	}
	else
	correctedRange = 0.0;*/
	
	}
	
	/*    // Returns the X displacement from starting location in inches.
	public double getFieldPositionX() {
	return fieldPosX;
	}
	
	public double getDistanceX() {
	return distanceX;
	}
	
	// Returns the Y displacement from starting location in inches.
	public double getFieldPositionY() {
	return fieldPosY;
	}
	
	public double getDistanceY() {
	return distanceY;
	}
	
	// X component of velocity vector in inches/sec.
	public double getVelocityX() {
	return velocityX;
	}
	
	// Y component of velocity vector in inches/sec.
	public double getVelocityY() {
	return velocityY;
	}
	*/
	
	// Returns the robot angular displacement from its original placement (in degrees).
	public double getAngle() {
		return robotAngleZ;
	}
	
	// Returns the robot angular rate (in degrees/sec).
	public double getAngleRate() {
		return robotAngleRateZ;
	}
	
	// Returns the current range from a detected object (in inches).
	public double getRange() {
		return correctedRange;
	}
	
    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

