package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends Subsystem {

    private Joystick m_Joystick;
    private double position;
    private Servo m_servo = new Servo(0);

    @Override
    public void robotInit() {
        DataLogManager.log("ClimberSubsystem in robotInit");
    }

    @Override
    public void autonomousInit() {
        DataLogManager.log("ClimberSubsystem in autonomousInit");
    }

    @Override
    public void teleopInit() {
        DataLogManager.log("ClimberSubsystem in teleopInit");
    }

    private static ClimberSubsystem mInstance;
    public static ClimberSubsystem getInstance(Joystick m_Joystick) {
      if (mInstance == null) {
        mInstance = new ClimberSubsystem(m_Joystick);
      }
      return mInstance;
    }

    public ClimberSubsystem(Joystick m_Joystick) {
        DataLogManager.log("ClimberSubsystem constructor");
        this.m_Joystick = m_Joystick;
    }

    @Override
    public void periodic() {
        position = ((m_Joystick.getRawAxis(3) + 1) / 2);
        m_servo.set(position);
        SmartDashboard.putNumber("Servo Position", position);    
    }
}
