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
import org.usfirst.frc4579.filters.FirstOrderLPF;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class driveTrain extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController leftMotor = RobotMap.driveTrainleftMotor;
    private final SpeedController rightMotor = RobotMap.driveTrainrightMotor;
    private final DifferentialDrive robotDrive = RobotMap.driveTrainrobotDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public int drive_direction = 1;
    public String pidMode = "rotate";
    public final double TURN_SPEED = 0.12;

    final double baseLine = 19.5; // inches
    
    private FirstOrderLPF vLeftLPF = new FirstOrderLPF(0.7);
    private FirstOrderLPF vRiteLPF = new FirstOrderLPF(0.7);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    /*public DriveTrain() {
    	super("DriveTrain", 2.0, 0.0, 0.0, 0.6);
		setAbsoluteTolerance(1.0);
		setInputRange(0.0, 360.0);
		setOutputRange(0.0,1.0);
		getPIDController().setContinuous(true);
		SmartDashboard.putData("PID Controller",getPIDController());
		SmartDashboard.putNumber("LEFT MOTOR OFFSET", 0.0);
		SmartDashboard.putNumber("RIGHT MOTOR SCALE FACTOR", 0.0);
    }*/
    
    protected static double limit(double num) {
        if (num > 1.0) {
          return 1.0;
        }
        if (num < -1.0) {
          return -1.0;
        }
        return num;
      }
    
    // Return true if the motors are commanded to zero.
    public boolean isNotMoving() {
    	return (leftMotor.get() == 0.0) && (rightMotor.get() == 0.0);
    }
    
    public void joeyStickDrive() { //The finest drive code known to man.
		
    	//Read the gyro and the driveStick.
		double gz = Robot.measurement.getAngleRate();
		gz = 0;
		double frwd = -Robot.oi.driveStick.getY();	//forward-back driveStick, speed control.
		double turn = Robot.oi.driveStick.getX();	    //left-right driveStick, turn control.

		//Lower limits for the driveStick, stop the motors.
		if (Math.abs(turn) < 0.04 && Math.abs(frwd) < 0.04) {
			turn = 0.0;
			frwd = 0.0;
			gz = 0.0;
		}
		
		//Decrease the low speed sensitivities of the driveStick.
		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd), 1.5);
		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn), 2.0);
		
		//Limit the control amount at high and low speeds, to avoid spinouts.
		double maxSens = 0.55;
		double minSens = 0.2;
		double sensitivity = maxSens - Math.abs(frwd2) * (maxSens - minSens);
		turn2 = turn2 * sensitivity;
		
		//Low pass filter the speed settings to the drive motors.
		double vLeft = vLeftLPF.filter(frwd2 + turn2 / 2.0);
		double vRite = vRiteLPF.filter(frwd2 - turn2 / 2.0);
		
		//Calculate the expected rotation rate.  93 in/sec (extrapolated full speed) converts the driveStick 
		//numbers to an expected speed value. The final equation is omega = (SpeedRite - SpeedLeft)/baseline.  
		//omega is rotation in deg/sec.
		double omega = Math.toDegrees((vRite - vLeft) * 93.0 / baseLine); 
		
		//Calculate the two wheel correction factor.
		double correction  = (omega - gz) * 0.008 / 2.0;
		double vRite2 = vRite + correction;
		double vLeft2 = vLeft - correction;
		
		//Normalize the wheel speeds to stay within +/-1.0;
		double magMax = Math.max(Math.abs(vRite2), Math.abs(vLeft2));
		if (magMax > 1.0) {
			vRite2 /= magMax;
			vLeft2 /= magMax;
		}
		
		//Set the two motor speeds.
		rightMotor.set(vRite2);
		leftMotor.set(vLeft2);
		
	}  
    
    // Drive each motor at the indicated speed.
    public void driveEachMotor (double leftMotorSpeed, double rightMotorSpeed) {
    	leftMotor .set(leftMotorSpeed);
    	rightMotor.set(rightMotorSpeed);
    }
    
    /*
    public void drive() {
    	double x = Robot.oi.driveStick.getX();//*drive_direction;
    	double y = Robot.oi.driveStick.getY();//*drive_direction;
    	double xsign = Math.signum(x);
    	double ysign = Math.signum(y);
    	final double slider = Robot.oi.driveStick.getRawAxis(2) / 2;
    	double sensitivity = 0.5 - slider;
    	//arcadeDriveCustom(x, y, true);
    	robotDrive.arcadeDrive(Math.pow(x*sensitivity, 2)*xsign/3, Math.pow(y*sensitivity, 2)*ysign*drive_direction, false);
    	SmartDashboard.putNumber("driveStick x-value", Robot.oi.driveStick.getX());
    	SmartDashboard.putNumber("driveStick y-value", Robot.oi.driveStick.getY());
    }
    */
    
    public void driveStraight(double speed) {
	    double halfCorrection = ((Robot.measurement.getAngleRate() * .006) + (Robot.measurement.getAngle() * .016)) /2.0;
	    Robot.driveTrain.driveEachMotor(speed + halfCorrection, speed - halfCorrection);
    }
    
    public void stop(){
    	leftMotor.stopMotor();
    	rightMotor.stopMotor();
    }
    //These methods reverse the direction that the robot drives in.
    public void reverseDrive() {
    	System.out.println("Setting drive to reverse");
    	drive_direction = -1;
    }
    public void forwardDrive() {
    	System.out.println("Setting drive to forward");
    	drive_direction = 1;
    }
    
    //This method continuously turns the robot at a set speed.
    public void continuousTurn(String dir) {
    	if (dir == "left") {
    		driveEachMotor(-TURN_SPEED, TURN_SPEED);
    	} else if (dir == "right") {
    		driveEachMotor(TURN_SPEED, -TURN_SPEED); 
    	}
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new drive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

