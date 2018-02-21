package org.usfirst.frc4579.Robot2018.commands;

import org.usfirst.frc4579.Robot2018.Robot;
import org.usfirst.frc4579.instrumentation.Instrumentation;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class calibrate extends Command{

	    // Called just before this Command runs the first time
	    protected void initialize() {
	    	setTimeout(10.0);
	    	Robot.measurement.reset();  //Reset the flow sensor accumulator.
	    }

	    // Called repeatedly when this Command is scheduled to run
	    protected void execute() {
	    	Robot.measurement.read(Robot.driveTrain.isNotMoving(), Instrumentation.timeNow());
	    	Robot.measurement.measure();
	    	
	    	Robot.driveTrain.joeyAutoDrive(0.3, 0.0);
	    	System.out.println(Robot.measurement.getFlowMotionX());
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
