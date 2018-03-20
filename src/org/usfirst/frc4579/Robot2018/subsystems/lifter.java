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

import org.usfirst.frc4579.Robot2018.Robot;
import org.usfirst.frc4579.Robot2018.RobotMap;
import org.usfirst.frc4579.Robot2018.commands.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class lifter extends PIDSubsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController liftMotor = RobotMap.lifterliftMotor;
    private final Encoder robotEncoder = RobotMap.lifterrobotEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Initialize your subsystem here
    public lifter() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID
        super("lifter", 0.1, 0.07, 0.0);
        setAbsoluteTolerance(0.2);
        getPIDController().setContinuous(false);
        LiveWindow.addActuator("lifter", "PIDSubsystem Controller", getPIDController());
        getPIDController().setOutputRange(-1.0, 1.0);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID

        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
        enable();
        
    }
    

    //---------------- Declaring Variables And Objects -----------------------//
    
    DigitalInput IRSOURCE = new DigitalInput(0);
    Counter IRSensor = new Counter(IRSOURCE);
    
    int height = 0;              // Stores what height the lifter is currently at
    private int lastHeight = 0;		// Stores the last height that will be used for the updateHeight method
    public int maxHeight = 100;			// The highest height the lifter can reach (Need to measure)
    double direction = 1;           // Stores whether the lifter is moving up or down
    int currentGoal = 0;         // Stores the height for the lifter to move to
    double heightGoals[] = {0,234,324};        // The height goals for the lifter
    
    
    public int getCounts(){
    	return IRSensor.get();
    }
    
    // Updates what the current height of the lifter is according to the IRSensor
    public void updateHeight(){
    	if((int)Math.signum(liftMotor.get()) == -1){
    		direction = -1;		// To tell whether to add or subtract the change in height
    	}
    	else{
    		direction = 1;
    	}
    	
    	// Updates the height
    	height -= (getCounts() - lastHeight) * direction;
    	
    	//Saves the last count for later use
    	lastHeight = getCounts();
    }
    
    public int getHeight(){
    	return height;
    }
    
    // Stops the motor at its current spot
    public void stopLift(){
    	//setSetpoint(height);
    	liftMotor.stopMotor();
    }
//    
//    //----------------- Methods for the incrementLift command ----------------//
//    // Increments the goal for the lifter when using the incrementLift command
//    public void incrementGoal() {
//    	if(currentGoal>=2) {
//    		currentGoal = 0;
//    	}
//    	else {
//    		currentGoal++;
//    	}
//    }
//
//    public void incrementLift() {
//    	setSetpoint(heightGoals[currentGoal]);
//    }
    
    
    public void resetCounter(){
    	height = 0;
    }
    
    
    //------------------------- Test Code ----------------------------//
    public void moveUp(){
    	liftMotor.set(-.75);
    }
    
    public void moveDown(){
    	liftMotor.set(.4);
    }
    
    // Goes back to the start position for the lifter
    public void toStart(){
    	while(!onTarget()){
    		setSetpoint(0);
    	}
    }
    
    
    public void autoDrop(){
    	if(height < 50){
    		Robot.lifter.moveUp();
    	}
    	else if(height >= 50){
    		Robot.gripper.eject(.5);
    	}
    }
    
    
    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new lift());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    @Override
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
        return robotEncoder.pidGet();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
    }

    @Override
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
        liftMotor.pidWrite(output);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
    }
}
