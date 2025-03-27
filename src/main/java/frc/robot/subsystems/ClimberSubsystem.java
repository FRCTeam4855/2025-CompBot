package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends Subsystem {

    private Joystick m_Joystick;
    private double position;
    private Servo m_servo = new Servo(0);
    public SparkMax ClimbSpark;
    public RelativeEncoder m_encoder;
    public SparkClosedLoopController m_PIDController;

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

        ClimbSpark = new SparkMax(ClimberConstants.kWinchCanId, MotorType.kBrushless);
        m_encoder = ClimbSpark.getEncoder();
        m_PIDController = ClimbSpark.getClosedLoopController();
        
        ClimbSpark.configure(Configs.ClimberSubsystem.climberWinchConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        position = ((m_Joystick.getRawAxis(3) + 1) / 2);
        m_servo.set(position);
        SmartDashboard.putNumber("Servo Position", position);    
    }
    
    public void ClimberWinchToSetpoint(int goalSetpoint) {
        m_PIDController.setReference(ClimberConstants.climberPos[goalSetpoint], SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
