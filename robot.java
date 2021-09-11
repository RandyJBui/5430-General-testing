// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//libraries for defining drivetrain type
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//Timed robot is the robot base
import edu.wpi.first.wpilibj.TimedRobot;
//Timer method for auton or use as a buffer between inputs
import edu.wpi.first.wpilibj.Timer;
//

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.GenericHID.Hand; //used to defined XboxController axis (probably wont be used).
//import edu.wpi.first.wpilibj.buttons.JoystickButton; //Only used for non integrated controllers, like Ps4.


import edu.wpi.first.wpilibj.PWMTalonSRX; //Test bed and not drive Train motors
import edu.wpi.first.wpilibj.PWMTalonFX; //Drive train motors


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private DifferentialDrive driveTrain;
  //bumper_map is bumper number  on controller
  private static final int leftBumper_Map = 5;
private static final int rightBumper_Map = 6;
//axis_Map is Axis number on controller
private static final int leftYAxis_Map = 1;
private static final int rightYAxis_Map = 5;
//controller usb_port
  private static final int controller_port = 0;
  //roboRio ports
  private static final int tLMotor_port = 0;
  private static final int bLMotor_port = 1;
  private static final int tRMotor_port = 2;
  private static final int bRMotor_port = 3;
  //used to modulate Motor speeds
  private static double percentspeed = 1;
  //used to limit times bumpers can be used
  private static int count = 0;

  private Timer m_timer = new Timer();
  private Joystick controller;

  //Auton Types, declare different ones here, can be chose on Driver Station 
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Sides on drive train
  private SpeedControllerGroup leftGroup;
  private SpeedControllerGroup rightGroup;
//motors that make up the drive train
 private SpeedController topLeft;
 private SpeedController botLeft;
 private SpeedController topRight;
 private SpeedController botRight;
 //private JoystickButton leftBumper = new JoystickButton(controller, leftBumper_Map); //Only used for non integrated controllers, like Ps4.
 //private JoystickButton rightBumper = new JoystickButton(controller, rightBumper_Map);


  
  @Override
  public void robotInit() {
    //chose auton mode from Driver station
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
   //declaring motors using ports
     topLeft =  new PWMTalonSRX(tLMotor_port);
     botLeft =  new PWMTalonSRX(bLMotor_port);
     topRight =  new PWMTalonSRX(tRMotor_port);
     botRight =  new PWMTalonSRX(bRMotor_port);
     //creating controller
     controller = new Joystick(controller_port);
     //grouping up motors
     leftGroup = new SpeedControllerGroup(topLeft, botLeft);
     rightGroup = new SpeedControllerGroup(topRight, botRight);
     //creating drive train
     driveTrain = new DifferentialDrive(leftGroup, rightGroup);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
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

  //𝐂𝐚𝐧 𝐜𝐡𝐨𝐨𝐬𝐞 𝐝𝐢𝐟𝐟𝐞𝐫𝐞𝐧𝐭 𝐢𝐧𝐬𝐭𝐚𝐧𝐜𝐞𝐬 𝐨𝐟 𝐀𝐮𝐭𝐨𝐧 𝐡𝐞𝐫𝐞, 𝐚𝐥𝐥 𝐰𝐞 𝐰𝐨𝐮𝐥𝐝 𝐡𝐚𝐯𝐞 𝐭𝐨 𝐝𝐨 𝐢𝐬 𝐝𝐞𝐜𝐥𝐚𝐫𝐞 𝐚 𝐬𝐜𝐚𝐧𝐧𝐞𝐫 𝐨𝐫 𝐜𝐫𝐞𝐚𝐭𝐞 𝐚 𝐉𝐅𝐫𝐚𝐦𝐞 𝐚𝐧𝐝 𝐡𝐚𝐯𝐞 𝐭𝐡𝐞 𝐝𝐫𝐢𝐯𝐞 𝐩𝐢𝐜𝐤 𝐚𝐧 𝐨𝐩𝐭𝐢𝐨𝐧
  //𝐂𝐨𝐮𝐥𝐝 𝐛𝐞 𝐮𝐬𝐞𝐟𝐮𝐥 𝐢𝐟 𝐰𝐞 𝐰𝐚𝐧𝐭𝐞𝐝 𝐭𝐨 𝐝𝐨 𝐝𝐢𝐟𝐟𝐞𝐫𝐞𝐧𝐭 𝐚𝐮𝐭𝐨𝐧 𝐜𝐨𝐝𝐞 𝐢𝐟 𝐰𝐞 𝐰𝐞𝐫𝐞 𝐠𝐨𝐢𝐧𝐠 𝐥𝐞𝐟𝐭, 𝐫𝐢𝐠𝐡𝐭, 𝐨𝐫 𝐜𝐞𝐧𝐭𝐞𝐫.
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        //
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

    //If user presses left bumper, cut power of motors by 20% of max, stopping at 20% max power.
    if(controller.getRawButton(leftBumper_Map) & (count != 4)) {
      //resets timer at every instance if condition is met
      m_timer.reset();
percentspeed = percentspeed - .2;
count++;
      //start the buffer timer
      m_timer.start();
        //Because of the fact that periodic runs at ~50 Hz we need a buffer so we dont send too many signals in one press.
      while(m_timer.get() < .5){
        System.out.println("wait time"); //Code test piece, can be removed before deployment.
      }
    }

    //If user presses right bumper, increments output by 20% of max. Will only activate if max power was decremented atleast once.
    if((controller.getRawButton(rightBumper_Map)) & (percentspeed != 1)) {
      m_timer.reset();
percentspeed = percentspeed + .2;
count--;
      m_timer.start();

      while(m_timer.get() < .5){
        System.out.println("wait time");
      }
    }
    //deadzone code, might have to adjust if controller is even more off center.
    if(controller.getRawAxis(leftYAxis_Map) > .05 | controller.getRawAxis(leftYAxis_Map) < -.05 | controller.getRawAxis(rightYAxis_Map) > .05 | controller.getRawAxis(rightYAxis_Map) < -.05){
    driveTrain.tankDrive((-controller.getRawAxis(leftYAxis_Map) * percentspeed),( -controller.getRawAxis(rightYAxis_Map) * percentspeed ));
    System.out.println("Right Speed : " + -controller.getRawAxis(rightYAxis_Map) * percentspeed); //Used for simulation confimation, can remove before deployment.
    System.out.println("Left Speed : " + -controller.getRawAxis(leftYAxis_Map) * percentspeed);
    }else{
      //makes sure that motors dont start when there is no input
      driveTrain.tankDrive(0, 0);
    }
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
