package org.usfirst.frc4579.Robot2018.commands;

import org.usfirst.frc4579.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class down extends Command {

    public down() {
        // Use requires() here to declare subsystem dependencies  //JGH you're missing this element for the PID.
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println("Going Down");
    	Robot.lifter.moveDown();
    	Robot.lifter.updateHeight();
    	SmartDashboard.putNumber("IRValue", (double)Robot.lifter.getHeight());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.oi.everyStick.getRawButtonReleased(9);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.lifter.stopLift();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
