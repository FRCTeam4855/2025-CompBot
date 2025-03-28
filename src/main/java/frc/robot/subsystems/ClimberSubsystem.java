package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DataLogManager;
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

    private Servo m_servo = new Servo(ClimberConstants.kWinchRatchetServo);
    public SparkMax m_winchSpark, m_rotateSpark;
    public RelativeEncoder m_winchEncoder, m_rotateEncoder;
    public SparkClosedLoopController m_winchPIDController, m_rotatePIDController;

    @Override
    public void robotInit() {
        DataLogManager.log("ClimberSubsystem in robotInit");
        WinchRatchetSetPosition(1);
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
        SmartDashboard.putNumber("Winch Encoder Value", m_winchEncoder.getPosition());
    }
    
    public void ClimberWinchDriveDirect(double speed) {
        m_winchSpark.set(speed);
    }
    
    public void ClimberIntakeDriveDirect(double speed) {
        m_rotateSpark.set(speed);
    }
    
    public void ClimberWinchToSetpoint(int goalSetpoint) {
        m_winchPIDController.setReference(ClimberConstants.climberPos[goalSetpoint], SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        System.out.println("Winch To Setpoint");
        System.out.println(ClimberConstants.climberPos[goalSetpoint]);
    }
    
    public void ClimberIntakeToSetpoint(int goalSetpoint) {
        m_rotatePIDController.setReference(ClimberConstants.rotatePos[goalSetpoint], SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        System.out.println("Intake To Setpoint");
        System.out.println(ClimberConstants.rotatePos[goalSetpoint]);
    }

    public void WinchRatchetSetPosition(int position) {
        m_servo.set(ClimberConstants.ratchetPos[position]);
    }
}
