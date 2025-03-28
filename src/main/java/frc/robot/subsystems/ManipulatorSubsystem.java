package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends Subsystem {

    public final SparkMax m_rightSparkMax;
    public final SparkMax m_leftSparkMax;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    private DigitalInput m_innerCoralSensor;
    private DigitalInput m_outerCoralSensor;
    public boolean innerSensor, outerSensor;
    public boolean useSensor;
    public boolean coralSensor;
    public boolean stopManipulator = false;

    @Override
    public void robotInit() {
        DataLogManager.log("ManipulatorSubsystem in robotInit");
        StopManipulator();
    }

    @Override
    public void autonomousInit() {
        DataLogManager.log("ManipulatorSubsystem in autonomousInit");
    }

    @Override
        public void teleopInit() {
        DataLogManager.log("ManipulatorSubsystem in teleopInit");
    }

    private static ManipulatorSubsystem mInstance;
    public static ManipulatorSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new ManipulatorSubsystem();
      }
      return mInstance;
    }

    public ManipulatorSubsystem() {
        m_rightSparkMax = new SparkMax(ManipulatorConstants.kRightManipulatorCanId, MotorType.kBrushless);
        m_leftSparkMax = new SparkMax(ManipulatorConstants.kLeftManipulatorCanId, MotorType.kBrushless);
        rightPIDController = m_rightSparkMax.getClosedLoopController();
        leftPIDController = m_leftSparkMax.getClosedLoopController();
        m_innerCoralSensor = new DigitalInput(0);
        m_outerCoralSensor = new DigitalInput(1);

        m_rightSparkMax.configure(Configs.ManipulatorSubsystem.rightManipulatorConfig, 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftSparkMax.configure(Configs.ManipulatorSubsystem.leftManipulatorConfig, 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        innerSensor = !m_innerCoralSensor.get();
        outerSensor = !m_outerCoralSensor.get();
        SmartDashboard.putBoolean("inner Coral Sensor", innerSensor);
        SmartDashboard.putBoolean("outer Coral Sensor", outerSensor);
    }

    public boolean isElevatorClear() {
        if (innerSensor || !stopManipulator) {
            return false;
        } else {
            return true;
        }
    }

    public void RunManipulator(double speed) {
        stopManipulator = false;
        m_rightSparkMax.set(speed);
        //m_leftSparkMax.set(speed);
    }

    public void StopManipulator() {
        m_rightSparkMax.set(0);
        stopManipulator = true;
    }

}