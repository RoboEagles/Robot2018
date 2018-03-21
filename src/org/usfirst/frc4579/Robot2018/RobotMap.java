// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4579.Robot2018;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveTrainleftMotor;
    public static SpeedController driveTrainrightMotor;
    public static DifferentialDrive driveTrainrobotDrive;
    public static SpeedController gripperrightGripper;
    public static SpeedController gripperleftGripper;
    public static SpeedController grippermovementMotor;
    public static SpeedController lifterliftMotor;
    public static Encoder lifterrobotEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainleftMotor = new PWMVictorSPX(0);
        LiveWindow.addActuator("driveTrain", "leftMotor", (PWMVictorSPX) driveTrainleftMotor);
        driveTrainleftMotor.setInverted(false);
        driveTrainrightMotor = new PWMVictorSPX(1);
        LiveWindow.addActuator("driveTrain", "rightMotor", (PWMVictorSPX) driveTrainrightMotor);
        driveTrainrightMotor.setInverted(true);
        driveTrainrobotDrive = new DifferentialDrive(driveTrainleftMotor, driveTrainrightMotor);
        LiveWindow.addActuator("driveTrain", "robotDrive", driveTrainrobotDrive);
        driveTrainrobotDrive.setSafetyEnabled(true);
        driveTrainrobotDrive.setExpiration(0.1);
        driveTrainrobotDrive.setMaxOutput(1.0);

        gripperrightGripper = new PWMVictorSPX(3);
        LiveWindow.addActuator("gripper", "rightGripper", (PWMVictorSPX) gripperrightGripper);
        gripperrightGripper.setInverted(false);
        gripperleftGripper = new PWMVictorSPX(2);
        LiveWindow.addActuator("gripper", "leftGripper", (PWMVictorSPX) gripperleftGripper);
        gripperleftGripper.setInverted(false);
        grippermovementMotor = new Spark(5);
        LiveWindow.addActuator("gripper", "movementMotor", (Spark) grippermovementMotor);
        grippermovementMotor.setInverted(false);
        lifterliftMotor = new Spark(4);
        LiveWindow.addActuator("lifter", "liftMotor", (Spark) lifterliftMotor);
        lifterliftMotor.setInverted(false);
//        lifterrobotEncoder = new Encoder(0, 1, false, EncodingType.k4X);
//        LiveWindow.addSensor("lifter", "robotEncoder", lifterrobotEncoder);
//        lifterrobotEncoder.setDistancePerPulse(1.0);
//        lifterrobotEncoder.setPIDSourceType(PIDSourceType.kRate);   //JGH  I think this should be kDistance.  If no PID, remove it.

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
