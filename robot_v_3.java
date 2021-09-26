// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//libraries for defining drivetrain type
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
//Timed robot is the robot base
import edu.wpi.first.wpilibj.TimedRobot;
//Timer method for auton or use as a buffer between inputs
import edu.wpi.first.wpilibj.Timer;
//
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.GenericHID.Hand; //used to defined XboxController axis (probably wont be used).
//import edu.wpi.first.wpilibj.buttons.JoystickButton; //Only used for non integrated controllers, like Ps4.
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMTalonSRX; //Test bed and not drive Train motors
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PWMTalonFX; //Drive train motors

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private DifferentialDrive driveTrain;
  // bumper_map is bumper number on controller
  private static final int leftBumper_Map = 5;
  private static final int rightBumper_Map = 6;
  //f310 buttons
  private static final int xButton = 3;
  private static final int bButton = 2;
  // axis_Map is Axis number on controller
  private static final int leftYAxis_Map = 1;
  private static final int rightYAxis_Map = 5;
  // controller usb_port
  private static final int controller_port = 0;
  private static final int controller_port_right = 1;
  private static final int controller_port_left = 2;
  // roboRio ports
  private static final int tLMotor_port = 0;
  private static final int bLMotor_port = 1;
  private static final int tRMotor_port = 2;
  private static final int bRMotor_port = 3;
  private static final int shooter_port = 4;
  private static final int elevator_port = 5;
  private static final int spinner_port = 6;
  // used to modulate Motor speeds and # of spins
  private static double percentspeed = 1;
  private static int spins = 1;
  //adjustable constants for pilots
  private static int spinConstant = 2;
  private static double MaxSpeed = 1;
  private static double PilotSpeed = 0.5;
  // used to limit times bumpers can be used
  private static int count = 0;
  
  private Timer m_timer = new Timer();
  private Joystick controller;
  private Joystick joystickright;
  private Joystick joystickleft;

  // Auton Types, declare different ones here, can be chose on Driver Station
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // subsystem motors
  private PWMTalonSRX spinner;
  private Spark shooter;
  private PWMTalonSRX elevator;
  // Sides on drive train
  private SpeedControllerGroup leftGroup;
  private SpeedControllerGroup rightGroup;
  // motors that make up the drive train
  private SpeedController topLeft;
  private SpeedController botLeft;
  private SpeedController topRight;
  private SpeedController botRight;
  // private JoystickButton leftBumper = new JoystickButton(controller,
  // leftBumper_Map); //Only used for non integrated controllers, like Ps4.
  // private JoystickButton rightBumper = new JoystickButton(controller,
  // rightBumper_Map);
  private Compressor compressor1;
  private Solenoid solenoid1;

  @Override
  public void robotInit() {
    // chose auton mode from Driver station
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // declaring motors using ports
    elevator = new PWMTalonSRX(elevator_port);
    shooter = new Spark(shooter_port);
    topLeft = new PWMTalonSRX(tLMotor_port);
    botLeft = new PWMTalonSRX(bLMotor_port); // !!!ＩＭＰＯＲＴＡＮＴ， ＩＦ ＤＥＰＬＯＹＩＮＧ ＩＮＴＯ ＲＯＢＯＴ ＳＷＡＰ ＴＨＩＳ ＴＯ ＰＷＭ!!!
    topRight = new PWMTalonSRX(tRMotor_port);
    botRight = new PWMTalonSRX(bRMotor_port);
    spinner = new PWMTalonSRX(spinner_port);
    compressor1 = new Compressor();
    // creating controller
    joystickright = new Joystick(controller_port_right);
    joystickleft = new Joystick(controller_port_left);
    controller = new Joystick(controller_port);
    // grouping up motors
    leftGroup = new SpeedControllerGroup(topLeft, botLeft);
    rightGroup = new SpeedControllerGroup(topRight, botRight);
    // creating drive train
    driveTrain = new DifferentialDrive(leftGroup, rightGroup);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */

  // 𝐂𝐚𝐧 𝐜𝐡𝐨𝐨𝐬𝐞 𝐝𝐢𝐟𝐟𝐞𝐫𝐞𝐧𝐭 𝐢𝐧𝐬𝐭𝐚𝐧𝐜𝐞𝐬 𝐨𝐟 𝐀𝐮𝐭𝐨𝐧
  // 𝐡𝐞𝐫𝐞, 𝐚𝐥𝐥 𝐰𝐞 𝐰𝐨𝐮𝐥𝐝 𝐡𝐚𝐯𝐞 𝐭𝐨 𝐝𝐨 𝐢𝐬 𝐝𝐞𝐜𝐥𝐚𝐫𝐞 𝐚
  // 𝐬𝐜𝐚𝐧𝐧𝐞𝐫 𝐨𝐫 𝐜𝐫𝐞𝐚𝐭𝐞 𝐚 𝐉𝐅𝐫𝐚𝐦𝐞 𝐚𝐧𝐝 𝐡𝐚𝐯𝐞 𝐭𝐡𝐞
  // 𝐝𝐫𝐢𝐯𝐞 𝐩𝐢𝐜𝐤 𝐚𝐧 𝐨𝐩𝐭𝐢𝐨𝐧
  // 𝐂𝐨𝐮𝐥𝐝 𝐛𝐞 𝐮𝐬𝐞𝐟𝐮𝐥 𝐢𝐟 𝐰𝐞 𝐰𝐚𝐧𝐭𝐞𝐝 𝐭𝐨 𝐝𝐨
  // 𝐝𝐢𝐟𝐟𝐞𝐫𝐞𝐧𝐭 𝐚𝐮𝐭𝐨𝐧 𝐜𝐨𝐝𝐞 𝐢𝐟 𝐰𝐞 𝐰𝐞𝐫𝐞 𝐠𝐨𝐢𝐧𝐠
  // 𝐥𝐞𝐟𝐭, 𝐫𝐢𝐠𝐡𝐭, 𝐨𝐫 𝐜𝐞𝐧𝐭𝐞𝐫.
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // autonomous code, foward left, foward right, backwards left, backwards right,
        // both fowards, both backwards, all for one second.
        m_timer.start();
        while (m_timer.get() < 1) {
          driveTrain.tankDrive(1, 0);
        }
        while ((m_timer.get() > 1) & (m_timer.get() < 2)) {
          driveTrain.tankDrive(0, 1);

        }
        while ((m_timer.get() > 2) & (m_timer.get() < 3)) {
          driveTrain.tankDrive(-1, 0);
        }
        while ((m_timer.get() > 3) & (m_timer.get() < 4)) {
          driveTrain.tankDrive(0, -1);
        }
        while ((m_timer.get() > 4) & (m_timer.get() < 5)) {
          driveTrain.tankDrive(1, 1);
        }
        while ((m_timer.get() > 5) & (m_timer.get() < 6)) {
          driveTrain.tankDrive(-1, -1);
        }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (controller.getPOV() == 0) {
      spins++;


      m_timer.reset();
      m_timer.start();
      while (m_timer.get() < .1) {
        System.out.println("waiting");
      }
      m_timer.stop();
      m_timer.reset();
      System.out.println("number of rotations: " + spins  );
    }

    if (( (controller.getPOV() == 180) && (spins > 1) ) ) {
      spins--;

      m_timer.reset();
      m_timer.start();
      while (m_timer.get() < .2) {
        System.out.println("waiting");
      }
      m_timer.stop();
      m_timer.reset();
      System.out.println("number of rotations: " + spins );
    }

    // If user presses left bumper, cut power of motors by 20% of max, stopping at
    // 20% max power.
    if (controller.getRawButton(leftBumper_Map) & (count != 4)) {
      // resets timer at every instance if condition is met
      m_timer.reset();
      percentspeed = percentspeed - .2;
      count++;
      // start the buffer timer
      m_timer.start();
      // Because of the fact that periodic runs at ~50 Hz we need a buffer so we dont
      // send too many signals in one press.
      while (m_timer.get() < .2) {
        System.out.println("Waiting"); // Code test piece, can be removed before deployment.
      }
      m_timer.stop();
      m_timer.reset();
    }

    // If user presses right bumper, increments output by 20% of max. Will only
    // activate if max power was decremented atleast once.
    if ((controller.getRawButton(rightBumper_Map)) & (percentspeed != 1)) {
      m_timer.reset();
      percentspeed = percentspeed + .2;
      count--;
      m_timer.start();

      while (m_timer.get() < .2) {
        System.out.println("Waiting");
      }
      m_timer.stop();
      m_timer.reset();
    }else{
      spinner.setSpeed(0);
    }
    if(controller.getRawButtonPressed(xButton)){
   
      m_timer.reset();
 
      m_timer.start();
 
      while(m_timer.get() <= (spins * spinConstant)){ // spin constant is a variable we need to test or calculate, essentially the time it takes for one color wheel rotation
        spinner.setSpeed(MaxSpeed);
        System.out.println("spinning");
      }
      m_timer.stop();
      m_timer.reset();
    }else{
      spinner.setSpeed(0);
    }

    if(controller.getRawButtonPressed(bButton)){
      while(!controller.getRawButtonReleased(bButton)){
      spinner.setSpeed(PilotSpeed); //vary the constant to find the best speed for copilot to get best accuracy.
      System.out.println("spinning");
      }
    }

    // deadzone code, might have to adjust if controller is even more off center.
    if (joystickleft.getY() > .05 | joystickleft.getY() < -.05 | joystickright.getY() > .05
        | joystickright.getY() < -.05 | controller.getY(Hand.kRight) > .05 | controller.getY(Hand.kLeft) > .5) {
      driveTrain.tankDrive((-joystickleft.getY() * percentspeed), (-joystickright.getY() * percentspeed));
      // System.out.println("Right Speed : " + -controller.getRawAxis(rightYAxis_Map)
      // * percentspeed);
       //Used for simulation confimation, can remove before
      // deployment.
      // System.out.println("Left Speed : " + -controller.getRawAxis(leftYAxis_Map) *
      // percentspeed);

      // in order to bypass the need for a whole jumble of code, where we would need
      // an if statement, just use controller axis in order to control elevator and
      // shooter,
      // must use absolute value of axis for shooter. reason why we dont use buttons:
      // use of another if statement will cancel or possibly stall updating code for
      // the previous part. to overcome that we could make a bunch of different else
      // if statements with every single possible combination, but this seems like the
      // fastest most efficient work around.

      //ᴍᴜꜱᴛ ᴛᴇꜱᴛ, ᴛʜᴇ ᴜꜱᴇ ᴏꜰ ʙᴜᴛᴛᴏɴꜱ + ꜱᴇɴᴅɪɴɢ ꜱᴘᴇᴇᴅ ᴠᴀʟᴜᴇꜱ, ᴄᴜʀʀᴇɴᴛ ᴛʜᴇᴏʀʏ ɪꜱ ᴛʜᴀᴛ ᴜꜱɪɴɢ ᴀɴ ɪꜰ
      // ꜱᴛᴀᴛᴇᴍᴇɴᴛ ʙᴜᴛᴛᴏɴ ᴡɪʟʟ ɪɴᴛᴇʀᴜᴘᴛ ᴛʜᴇ ꜰʟᴏᴡ ᴏꜰ ᴛʜᴇ ᴄᴏᴅᴇ, ᴀɴᴅ ᴛʜᴇ ꜱᴘᴇᴇᴅ ᴡɪʟʟ ꜱᴛᴀʟʟ, ʙᴜᴛ ᴄᴏɴꜱɪᴅᴇʀɪɴɢ ʜᴏᴡ ꜰᴀꜱᴛ ᴘᴇʀɪᴏᴅɪᴄ ɪꜱ, ᴍᴀʏʙᴇ ᴛʜᴇ ꜱᴛᴀʟʟ ᴡᴏɴᴛ ʜᴀᴠᴇ ᴀ ɢʀᴇᴀᴛ ᴄᴏɴꜱᴇQᴜᴇɴᴄᴇ?

      elevator.setSpeed(controller.getY(Hand.kLeft));
      shooter.setSpeed(controller.getY(Hand.kRight));
    } else {
      // makes sure that motors dont start when there is no input
      driveTrain.tankDrive(0, 0);
      elevator.setSpeed(0); // change values if we want passive charging of either elevator or shooter.
      shooter.setSpeed(0);
      
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
