package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */




public class Robot extends TimedRobot {

  private static final byte[] _ports = {0,1,2,3,4,5}; //port numbers in an array
  private static final byte[] _buttons = {0,1,2,3,4}; //Private makes it so that only this class can use these variables, static means that its only initialized once, and final makes it so the variable doesnt change.
  private static final byte[] _axis = {1,2,3,4};
 //trying out arrays in order to manage complexity.

final Compressor _compressor = new Compressor(_ports[0]);
final DoubleSolenoid _dSolenoid = new DoubleSolenoid(_ports[0], _ports[1]); // you can find these values on the pcm
final DigitalInput _limitSwitch = new DigitalInput(_ports[0]); //this is the DIO port the limit switch is plugged into.
//put color detector code here

final WPI_TalonSRX _motor = new WPI_TalonSRX(_ports[3]); //the motor controller in our testbed, you can find the port number in the Talon Tuner
final Joystick _joystick = new Joystick(_ports[0]); //joystick, port number is seperate to the compressor or solenoid, you can change this in the driver station.

private static final String turnAuto = "turnAuto"; //used to pick autonomous code in the driverstation
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    _dSolenoid.set(Value.kForward); //sets the solenoid to foward, in this case it is whichever side is assigned to port 0.

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { // periodics mean that the code repeats, so right now the motor is set to run foward forever, if we want to do steps, it'll probably better to use timers in autoninit
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
        case turnAuto: //have to set autonomous option in driver station.

        _motor.set(1); // activate one motor.
        break;
      case kDefaultAuto:
      default: //default is what plays when nothing else is picked.
        _compressor.start();
        break;
       

    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

if(_compressor.getPressureSwitchValue()){ //reads pressure switch value, if it is low (which it always is on the test bed) the following segment will run.
_compressor.stop(); //most likely this will be _compressor.start(); but this is loud.
}

if( (_joystick.getRawAxis(_axis[0]) > .02) | (_joystick.getRawAxis(_axis[0]) < -.02) ){ //deadzone code, can adjust to get the smallest amount w/o drift.
_motor.set(_joystick.getRawAxis(_axis[0]));
}else{
  _motor.set(0); //in any instance that motor value is set to anything other than 0, make sure that it somehow updates back to zero, this is just a safety precaution.
}

if(_joystick.getRawButton(_buttons[0])){
_dSolenoid.toggle(); //swaps direction of the solenoid.
  }
  
  if(_limitSwitch.get()){ //basically saying "is limit switch tripped?"
  _motor.set(1); //if Limit switch is pressed, set motor to 1
  }else{
    _motor.set(0);
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
