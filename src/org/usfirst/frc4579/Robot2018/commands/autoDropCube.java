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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc4579.Robot2018.Robot;

/**
 *
 */
public class autoDropCube extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public autoDropCube() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    Timer timer = new Timer();
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	timer.stop();
    	timer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	SmartDashboard.putNumber("IR Value: ", Robot.lifter.getHeight());
    	if(Robot.lifter.getHeight() < 50) Robot.lifter.moveUp();
    	else{
    		timer.start();
    		Robot.lifter.stopLift();
    		Robot.gripper.eject(1); 
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return timer.get() > 3;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	Robot.lifter.stopLift();
    	Robot.gripper.stopGripper();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }
}
