package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
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

    private Servo m_servo = new Servo(ClimberConstants.kWinchRatchetServo);
    public SparkMax m_winchSpark, m_rotateSpark;
    public RelativeEncoder m_winchEncoder, m_rotateEncoder;
    public SparkClosedLoopController m_winchPIDController, m_rotatePIDController;

    @Override
    public void robotInit() {
        DataLogManager.log("ClimberSubsystem in robotInit");
        WinchRatchetSetPosition(ClimberConstants.kWinchRatchetReleased);
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
    public static ClimberSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new ClimberSubsystem();
      }
      return mInstance;
    }

    public ClimberSubsystem() {
        DataLogManager.log("ClimberSubsystem constructor");

        m_winchSpark = new SparkMax(ClimberConstants.kWinchCanId, MotorType.kBrushless);
        m_winchEncoder = m_winchSpark.getEncoder();
        m_winchPIDController = m_winchSpark.getClosedLoopController();
        m_winchSpark.configure(Configs.ClimberSubsystem.climberWinchConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        m_rotateSpark = new SparkMax(ClimberConstants.kRotateCanId, MotorType.kBrushless);
        m_rotateEncoder = m_rotateSpark.getEncoder();
        m_rotatePIDController = m_rotateSpark.getClosedLoopController();
        m_rotateSpark.configure(Configs.ClimberSubsystem.climberRotateConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {        
    }
    
    public void ClimberWinchToSetpoint(int goalSetpoint) {
        m_winchPIDController.setReference(ClimberConstants.climberPos[goalSetpoint], SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    
    public void ClimberIntakeToSetpoint(int goalSetpoint) {
        m_rotatePIDController.setReference(ClimberConstants.rotatePos[goalSetpoint], SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void WinchRatchetSetPosition(double position) {
        m_servo.set(position);
    }
}
