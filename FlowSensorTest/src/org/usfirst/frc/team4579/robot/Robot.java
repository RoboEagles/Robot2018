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
	SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftRear);
	SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightRear);
	DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);
//	MecanumDrive drive = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
	Joystick stick = new Joystick(0);
	FlowDeck motion = new FlowDeck(SPI.Port.kOnboardCS0, 10);		 
	
	
	// Define the FlowDeck motion sensor via the SPI interface.
	/* This class is for the Flow Breakout board from Bitcraze.io.  The chip has the
	 * following registers, but they are not well defined in the datasheet.
	 * Address  Information   		Reset Value
		 0x00    Product_ID          0x49  RO
		 0x01    Revision_ID         0x00  RO
		 0x02    Motion              0x00  R/W
		 0x03    Delta_X_L           0x00  RO
		 0x04    Delta_X_H           0x00  RO
		 0x05    Delta_Y_L           0x00  RO
		 0x06    Delta_Y_H           0x00  RO
		 0x07    Squal               0x00  RO
		 0x08    RawData_Sum         0x00  RO
		 0x09    Maximum_RawData     0x00  RO
		 0x0A    Minumum_RawData     0x00  RO
		 0x0B    Shutter_Lower       0x00  RO
		 0x0C    Shutter_Upper       0x00  RO
		 0x5F    Inverse_Product_ID  0xB6  RO
		 Instructions from Bitcraze engineers indicate that we have a bad reading
		 if Squal is < 0x19(25) && if Shutter_Upper is = 0x1F(31).
	 */
	public class FlowDeck extends SensorBase {
		
		public int deltaX, deltaY, squal, shutter;
		public boolean goodSensor = false;
		private int oldX, oldY = 0;
		
		//Constructor.
		public FlowDeck(SPI.Port port, int chipSelectPort) {
			//Create the SPI port.
			SPI spiFlow = new SPI(port);
			//Create the Digital ChipSelect output bit.
			DigitalOutput cs = new DigitalOutput(chipSelectPort);
			//Initialize the port and the sensor.
			goodSensor = initSequence(cs);
		}
		
		//Primary functional method for the flow sensor.  The calling program should
		//  execute this method, and if it returns true, access the data elements
		//  directly, like: count = flowSensor.deltaX;
		public boolean readMotionCount() {
			if (registerRead(0x02) & (byte)0x80) = (byte)0x80) {
				byte dXL = registerRead(0x03);
				byte dXH = registerRead(0x04);
				byte dYL = registerRead(0x05);
				byte dYH = registerRead(0x06);
				squal = registerRead(0x07);
				shutter = registerRead(0x0C);
				deltaX = ((int)dXH << 8) | dXL;
				deltaY = ((int)dYH << 8) | dYL;
				if ((shutter = 31) & (squal < 25)) {
					deltaX = oldX;
					deltaY = oldY;
				}
				oldX = deltaX;
				oldY = deltaY;
				System.out.printf(" %0X  %0X  %0X  %0X \n", deltaX, deltaY, squal, shutter);
				return true;
			}
			else {
				deltaX, deltaY, squal, shutter = 0;
				return false;
			}
		}
		
		
		// Write a register via the SPI port.
		private void registerWrite(byte reg, byte value) {
			private ByteBuffer flowdata = new ByteBuffer.allocate(2);
			reg |= (byte)0x80;
			flowdata.put(0, reg);
			flowdata.put(1, value);
			cs.set(false);
			for (int i=0; i<11000; i++) {}  //delay 50 usec.
			spiFlow.write(flowdata, 2);
			for (int i=0; i<11000; i++) {}  //delay 50 usec.
			cs.set(true);
			for (int i=0; i<44000; i++) {}  //delay 200 usec.
//			System.out.printf("registerWrite buffer = [0]%02X [1]%02X\n", flowdata.get(0), flowdata.get(1));
		}
		
		// Read a register byte via the SPI port.
		private byte registerRead(byte reg) {
			private ByteBuffer flowdata = new ByteBuffer.allocate(2);
			reg &= 0x7F;
			flowdata.put(0, reg);
			flowdata.put(1, 0);
			cs.set(false);
			for (int i=0; i<11000; i++) {}  //delay 50 usec.
			spiFlow.write(flowdata, 1);
			for (int i=0; i<11000; i++) {}  //delay 50 usec.			
			spiFlow.read(true, flowdata, 1);
			for (int i=0; i<44000; i++) {}  //delay 200 usec.
			cs.set(true);
//			System.out.printf("registerRead buffer = [0]%02X [1]%02X\n", flowdata.get(0), flowdata.get(1));
			return flowdata.get(0);
		}
		
		// Initialize the SPI Port and the sensor chip.
		private boolean initSequence(DigitalOutput cs) {
			//Configure these settings to match SPI Mode 3. (See Wikipedia)
			spiFlow.setClockRate(2000000); //2 MHz
			spiFlow.setMSBFirst();
			spiFlow.setSampleDataOnRising();
			spiFlow.setClockActiveLow();
			spiFlow.setChipSelectActiveLow();
			spiFlow.stopAuto();
			System.out.println("SPI port is initialized.")
			//Reset the sensor SPI bus.
			cs.set(true);
			Timer.delay(0.001);
			cs.set(false);
			Timer.delay(0.001);
			cs.set(true);
			Timer.delay(0.001);
			
			//Initialize the sensor chip.
			// Power on reset
			registerWrite((byte)0x3A, (byte)0x5A);
			Timer.delay(.005);

			// Test the SPI communication, checking chipId and inverse chipId
			byte Product_ID = registerRead((byte)0x00);
			byte Inverse_Product_ID = registerRead((byte)0x5F);
			System.out.printf("*** ChipID 49: %02X, InverseChipID B6: %02H\n", Product_ID, Inverse_Product_ID)
			if (Product_ID !=(byte)0x49 && Inverse_Product_ID !=(byte)0xB6) {
				System.out.println("Sensor chip did not initialize.");
				return false;
			}
			// Reading the motion registers one time. the data isn't used.
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
			registerWrite((byte)0x65,(byte)0x67);
			registerWrite((byte)0x66,(byte)0x08);
			registerWrite((byte)0x63,(byte)0x70);
			registerWrite((byte)0x7F,(byte)0x15);
			registerWrite((byte)0x48,(byte)0x48);
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
			Timer.delay(0.01);
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
			registerWrite((byte)0x7F,(byte)0x00);
			registerWrite((byte)0x5A,(byte)0x10);
			registerWrite((byte)0x54,(byte)0x00);
			System.out.println("Sensor chip is initialized.")
			return true;
		}
		
	} //End of class definition.
	
		


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
		
		System.out.println(motion.goodSensor);
		if (!motion.goodSensor) {System.out.println("Chip Initialization failed!");}

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
		if (motion.goodSensor) {
			System.out.printf("** %6d  %6d  %6d  %6d\n",motion.deltaX, motion.deltaY, motion.squal, motion.shutter);
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
