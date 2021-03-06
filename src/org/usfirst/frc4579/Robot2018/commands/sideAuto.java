// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4579.Robot2018.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc4579.Robot2018.Robot;
import org.usfirst.frc4579.Robot2018.commands.autonomous;

/**
 *
 */
public class sideAuto extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public sideAuto() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

	public int step,   // Stores what step in the path it is at
				straightLocation;
	public double turnLocation,
					targetAngle;
	public double[] directions;
	
	public boolean firstStart = true;
	
	// The robots distance from its goal
	double distance;
	
	Timer timer = new Timer();
	
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		step = 0;
		turnLocation = 0.0;
		
		
    	// Gets the game data needed for the autonomous period
    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
		System.out.println(gameData);
		
    	if(!autonomous.sideSwitch.get()){
    		if(gameData.charAt(0) == 'R'){
    			// Pathway for when the robot is on the right and our switch is on the right
    			SmartDashboard.putString("Auto Config: ", "Switch to the Right and Robot to the Right");
    			directions = new double[] {1.0,90,-1,0};
    		}
    		else{
    			SmartDashboard.putString("Auto Config: ", "Switch to the Left and Robot to the Right");
    			directions = new double[]{1.0,90,3.0,90,-1,0};
    		}
    	}
    	else {
    		if(gameData.charAt(0) == 'L'){
    			// Pathway for when the robot is on the left and our switch is on the left
    			SmartDashboard.putString("Auto Config: ", "Switch to the Left and Robot to the Left");
    			directions = new double[] {3.0,-90,-1,0};
    		}
    		else{
    			SmartDashboard.putString("Auto Config: ", "Switch to the Right and Robot to the Left");
    			directions = new double[]{4.0,-90,3.0,-90,-1,0};
    		}
    	}
		
		timer.reset();
		timer.start();
	}
	
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		// The part of the code where it drives straight
		if(step%2 == 0){
			if(directions[step] < 0 && timer.get() < Math.abs(directions[step])){
				Robot.gripper.spinLeft(1);
				Robot.gripper.spinRight(1);
				if(timer.get() > .35) Robot.gripper.openGripper();
			}
			// Drives the robot straight until it reaches the goal
			if(timer.get() < directions[step] && directions[step] > 0){
				Robot.driveTrain.driveStraightReference(.4,turnLocation);
				System.out.println("Time: " + timer.get());
			}
		}
		
		// The part of the code where it is turning
		else if (step%2 == 1){
			distance = targetAngle - turnLocation;   //JGH  we should rethink how we do this.
			
			System.out.printf("  %d   %d\n", targetAngle, turnLocation);
			
			if(Math.abs(distance) > .5){
				// Turns the robot
				Robot.driveTrain.joeyStickDrive((-1.0 * Math.signum(directions[step])), 0);
				
				// Updates the robots current angle
				turnLocation = Robot.measurement.getAngle();	
				
				System.out.println("Turn Location: " + turnLocation);
				System.out.println("Distance: " + distance);
			}
		}
		
	}
	
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if(directions[step] == 0){
			return true;
		}
		else if(step%2 == 0 && timer.get() > Math.abs(directions[step])){
			// Resets the timer so that the next run will start at 0 seconds
			timer.stop();
			timer.reset();
			// Stops the driveTrain
			Robot.driveTrain.stop();
			
			// Stops the gripper
			Robot.gripper.stopGripper();
			
			// Updates which part of the path it is at
			step++;
			
			// The angle to be used for the turning part of the code
			
			distance = targetAngle - turnLocation;
		}
		
		else if(step%2 == 1 && Math.abs(distance) < .5){
			// Stops and updates where the robot is in the path
			Robot.driveTrain.stop();
			timer.reset();
			timer.start();
			step++;
		}
		return false;
	}
	 
	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.stop();
		System.out.println("Side Auto End*****");
		step = 0;
		turnLocation = 0;
	}
	
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
