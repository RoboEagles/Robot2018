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

import org.usfirst.frc4579.Robot2018.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc4579.Robot2018.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public Joystick driveStick;
    public JoystickButton leftGripperButton;
    public JoystickButton openButton;
    public JoystickButton ejectButton;
    public JoystickButton rightGripperButton;
    public JoystickButton closeButton;
    public Joystick everyStick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        everyStick = new Joystick(1);
        
        rightGripperButton = new JoystickButton(everyStick, 2);
        rightGripperButton.whileHeld(new moveRightGripper());
        ejectButton = new JoystickButton(everyStick, 7);
        ejectButton.whileHeld(new ejectBox());
        openButton = new JoystickButton(everyStick, 1);
        openButton.whenPressed(new openGripper());
        openButton.whenReleased(new closeGripper());
        leftGripperButton = new JoystickButton(everyStick, 3);
        leftGripperButton.whileHeld(new moveLeftGripper());
        driveStick = new Joystick(0);
        


        // SmartDashboard Buttons
        SmartDashboard.putData("moveLeftGripper", new moveLeftGripper());
        SmartDashboard.putData("drive", new drive());
        SmartDashboard.putData("lift", new lift());
        SmartDashboard.putData("ejectBox", new ejectBox());
        SmartDashboard.putData("sideAuto", new sideAuto());
        SmartDashboard.putData("autonomous", new autonomous());
        SmartDashboard.putData("centerAuto", new centerAuto());
        SmartDashboard.putData("openGripper", new openGripper());
        SmartDashboard.putData("moveRightGripper", new moveRightGripper());
        SmartDashboard.putData("incrementLift", new incrementLift());
        SmartDashboard.putData("closeGripper", new closeGripper());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        JoystickButton liftButton = new JoystickButton(everyStick, 8);
        liftButton.whileHeld(new lift());
        JoystickButton dropButton = new JoystickButton(everyStick, 9);
        dropButton.whileHeld(new down());
        
        XboxController xbox = new XboxController(3);
        
        
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getdriveStick() {
        return driveStick;
    }

    public Joystick geteveryStick() {
        return everyStick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

