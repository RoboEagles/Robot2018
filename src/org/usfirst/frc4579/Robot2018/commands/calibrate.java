package org.usfirst.frc4579.Robot2018.commands;

import org.usfirst.frc4579.Robot2018.Robot;
import org.usfirst.frc4579.instrumentation.Instrumentation;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class calibrate extends Command{
	public double speed = 0.0;

	    // Called just before this Command runs the first time
	    protected void initialize() {
	    	setTimeout(8.0);
	    	Robot.measurement.resetMPU();  //Reset the flow sensor accumulator.
	    }

	    // Called repeatedly when this Command is scheduled to run
	    protected void execute() {
	    	Robot.measurement.measure();
	    	speed += 0.04;
	    	if (speed > 0.5) speed = 0.5;
	    	Robot.driveTrain.driveStraight(speed);
	    }

	    // Make this return true when this Command no longer needs to run execute()
	    protected boolean isFinished() {
	        return isTimedOut();
	    }

	    // Called once after isFinished returns true
	    protected void end() {
	    	Robot.driveTrain.stop();
	    }

	    // Called when another command which requires one or more of the same
	    // subsystems is scheduled to run
	    protected void interrupted() {
	    	end();
	    }
	

}
