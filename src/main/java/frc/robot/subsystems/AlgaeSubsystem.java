package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Configs;

public class AlgaeSubsystem extends Subsystem {

    public final SparkMax m_intakeSparkMax;
    public final SparkMax m_armSparkMax;
    public final AbsoluteEncoder m_armEncoder;
    public final SparkClosedLoopController m_intakePIDController;
    public final SparkClosedLoopController m_armPIDController;
    private boolean floorPickup;

    @Override
    public void robotInit() {
        DataLogManager.log("AlgaeSubsystem in robotInit");
    }

    @Override
    public void autonomousInit() {
        DataLogManager.log("AlgaeSubsystem in autonomousInit");
    }

    @Override
    public void teleopInit() {
        DataLogManager.log("AlgaeSubsystem in teleopInit");
        ArmToPosition(0);
    }

    private static AlgaeSubsystem mInstance;
    public static AlgaeSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new AlgaeSubsystem();
      }
      return mInstance;
    }

    public AlgaeSubsystem() {
        
        m_intakeSparkMax = new SparkMax(AlgaeConstants.kIntakeCanId, MotorType.kBrushless);
        m_armSparkMax = new SparkMax(AlgaeConstants.kArmCanId, MotorType.kBrushless);
        m_armEncoder = m_armSparkMax.getAbsoluteEncoder();
        m_intakePIDController = m_intakeSparkMax.getClosedLoopController();
        m_armPIDController = m_armSparkMax.getClosedLoopController();
        floorPickup = false;
        m_intakeSparkMax.configure(Configs.AlgaeSubsystem.algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armSparkMax.configure(Configs.AlgaeSubsystem.algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
        SmartDashboard.putBoolean("Floor Pickup Mode", floorPickup);
    }

    public void IntakeAlgae() {
        if (floorPickup) {
            m_intakeSparkMax.set(-AlgaeConstants.kAlgaeSpeed);
        } else {
            m_intakeSparkMax.set(AlgaeConstants.kAlgaeSpeed);
        }
    }

    public void DeliverAlgae() {
        if (floorPickup) {
            m_intakeSparkMax.set(AlgaeConstants.kAlgaeSpeed);
        } else {
            m_intakeSparkMax.set(-AlgaeConstants.kAlgaeSpeed);
        }
    }

    public void StopIntake() {
        m_intakeSparkMax.set(0);
    }

    public void SetFloorPickup() {
        floorPickup = true;
    }

    public void SetReefPickup() {
        floorPickup = false;
    }

    public void TogglePickup() {
        floorPickup = !floorPickup;
    }
    public void ArmToPosition(int goalPosition) {
        SmartDashboard.putNumber("Goal Arm Position", goalPosition);
        m_armPIDController.setReference(AlgaeConstants.algaePos[goalPosition], SparkMax.ControlType.kPosition);
    }
}
