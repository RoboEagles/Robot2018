/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4579.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	SpeedController leftFront = new Victor(0);
	SpeedController rightFront = new Victor(3);
	SpeedController leftRear = new Victor(1);
	SpeedController rightRear = new Victor(2);
	MecanumDrive drive = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
	Joystick stick = new Joystick(0);
	
	// Define the FlowDeck motion sensor via the SPI interface.
	public class FlowDeck extends SensorBase  {
		//Create the SPI port.
		SPI flow = new SPI(SPI.Port.kOnboardCS0); 
		int count, accum = 0;
		byte[] flowdata = new byte[2];
		int deltaX, deltaY = 0;
		System.out.println("Define th SPI port.");

		// Write a register via the SPI port.
		public void registerWrite(byte reg, byte value)  {
			reg |=(byte)0x80;
			flowdata[0] = reg;
			flowdata[1] = value;
			flow.write(flowdata, 2);
		}
		
		// Read a register via the SPI port.
		public byte registerRead(byte reg)  {
			reg |=(byte)0x80;
			flow.read(true, flowdata, 1);
			return flowdata[0];
		}
		
		// Read the motion counts from the sensor.
		public void readMotionCount()  {
			this.registerRead((byte)0x02);
			registerRead((byte)0x02);
			deltaX = ((int)registerRead((byte)0x04) << 8) | registerRead((byte)0x03);
			deltaY = ((int)registerRead((byte)0x06) << 8) | registerRead((byte)0x05);
			System.out.print(deltaX); System.out.print("    ");  System.out.println(deltaY);
		}
		
		// Initialize the SPI Port.
		public boolean init() {
			//Configure these settings to match SPI Mode 3. (See Wikipedia)
			flow.setClockRate(2000000); //2 MHz
			flow.setMSBFirst();
			flow.setSampleDataOnRising();
			flow.setClockActiveLow();
			flow.setChipSelectActiveLow();
			flow.stopAuto();
			
			//Initialize the sensor chip.
			// Power on reset
			registerWrite((byte)0x3A, (byte)0x5A);
			Timer.delay(.005);

			// Test the SPI communication, checking chipId and inverse chipId
			byte chipId = registerRead((byte)0x00);
			byte dIpihc = registerRead((byte)0x5F);

			System.out.println(chipId);
			System.out.println(dIpihc);
			if (chipId !=(byte)0x49 && dIpihc !=(byte)0xB8) return false;

			// Reading the motion registers one time
			registerRead((byte)0x02);
			registerRead((byte)0x03);
			registerRead((byte)0x04);
			registerRead((byte)0x05);
			registerRead((byte)0x06);
			Timer.delay(.001);

			//Initialize the chip's registers.
			registerWrite((byte)0x7F,(byte)0x00);
			registerWrite((byte)0x61,(byte)0xAD);
			registerWrite((byte)0x7F,(byte)0x03);
			registerWrite((byte)0x40,(byte)0x00);
			registerWrite((byte)0x7F,(byte)0x05);
			registerWrite((byte)0x41,(byte)0xB3);
			registerWrite((byte)0x43,(byte)0xF1);
			registerWrite((byte)0x45,(byte)0x14);
			registerWrite((byte)0x5B,(byte)0x32);
			registerWrite((byte)0x5F,(byte)0x34);
			registerWrite((byte)0x7B,(byte)0x08);
			registerWrite((byte)0x7F,(byte)0x06);
			registerWrite((byte)0x44,(byte)0x1B);
			registerWrite((byte)0x40,(byte)0xBF);
			registerWrite((byte)0x4E,(byte)0x3F);
			registerWrite((byte)0x7F,(byte)0x08);
			registerWrite((byte)0x65,(byte)0x20);
			registerWrite((byte)0x6A,(byte)0x18);
			registerWrite((byte)0x7F,(byte)0x09);
			registerWrite((byte)0x4F,(byte)0xAF);
			registerWrite((byte)0x5F,(byte)0x40);
			registerWrite((byte)0x48,(byte)0x80);
			registerWrite((byte)0x49,(byte)0x80);
			registerWrite((byte)0x57,(byte)0x77);
			registerWrite((byte)0x60,(byte)0x78);
			registerWrite((byte)0x61,(byte)0x78);
			registerWrite((byte)0x62,(byte)0x08);
			registerWrite((byte)0x63,(byte)0x50);
			registerWrite((byte)0x7F,(byte)0x0A);
			registerWrite((byte)0x45,(byte)0x60);
			registerWrite((byte)0x7F,(byte)0x00);
			registerWrite((byte)0x4D,(byte)0x11);
			registerWrite((byte)0x55,(byte)0x80);
			registerWrite((byte)0x74,(byte)0x1F);
			registerWrite((byte)0x75,(byte)0x1F);
			registerWrite((byte)0x4A,(byte)0x78);
			registerWrite((byte)0x4B,(byte)0x78);
			registerWrite((byte)0x44,(byte)0x08);
			registerWrite((byte)0x45,(byte)0x50);
			registerWrite((byte)0x64,(byte)0xFF);
			registerWrite((byte)0x65,(byte)0x1F);
			registerWrite((byte)0x7F,(byte)0x14);
			registerWrite((byte)0x65,(byte)0x60);
			registerWrite((byte)0x66,(byte)0x08);
			registerWrite((byte)0x63,(byte)0x78);
			registerWrite((byte)0x7F,(byte)0x15);
			registerWrite((byte)0x48,(byte)0x58);
			registerWrite((byte)0x7F,(byte)0x07);
			registerWrite((byte)0x41,(byte)0x0D);
			registerWrite((byte)0x43,(byte)0x14);
			registerWrite((byte)0x4B,(byte)0x0E);
			registerWrite((byte)0x45,(byte)0x0F);
			registerWrite((byte)0x44,(byte)0x42);
			registerWrite((byte)0x4C,(byte)0x80);
			registerWrite((byte)0x7F,(byte)0x10);
			registerWrite((byte)0x5B,(byte)0x02);
			registerWrite((byte)0x7F,(byte)0x07);
			registerWrite((byte)0x40,(byte)0x41);
			registerWrite((byte)0x70,(byte)0x00);
			Timer.delay(0.1);
			registerWrite((byte)0x32,(byte)0x44);
			registerWrite((byte)0x7F,(byte)0x07);
			registerWrite((byte)0x40,(byte)0x40);
			registerWrite((byte)0x7F,(byte)0x06);
			registerWrite((byte)0x62,(byte)0xf0);
			registerWrite((byte)0x63,(byte)0x00);
			registerWrite((byte)0x7F,(byte)0x0D);
			registerWrite((byte)0x48,(byte)0xC0);
			registerWrite((byte)0x6F,(byte)0xd5);
			registerWrite((byte)0x7F,(byte)0x00);
			registerWrite((byte)0x5B,(byte)0xa0);
			registerWrite((byte)0x4E,(byte)0xA8);
			registerWrite((byte)0x5A,(byte)0x50);
			registerWrite((byte)0x40,(byte)0x80);
			return true;

		}

		@Override
		public void initSendable(SendableBuilder builder) {
			// TODO Auto-generated method stub
			
		}
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		System.out.println("*** Robot Init running.")
		
		//Initialize the SPI Port
		System.out.println("*** Starting FlowDeck Initialization...")
		FlowDeck robotFlow = new FlowDeck();
		boolean goodInit = robotFlow.init();
		System.out.println(goodInit);
		if (!goodInit) {System.out.println("Chip Init failed!");}

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		drive.driveCartesian(stick.getY(), stick.getX(), 0.0);

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
