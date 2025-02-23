package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Configs;

public class AlgaeSubsystem extends SubsystemBase {

    public final SparkMax m_intakeSparkMax;
    public final SparkMax m_armSparkMax;
    public final AbsoluteEncoder m_armEncoder;
    public final SparkClosedLoopController m_intakePIDController;
    public final SparkClosedLoopController m_armPIDController;

    public AlgaeSubsystem() {
        
        m_intakeSparkMax = new SparkMax(AlgaeConstants.kIntakeCanId, MotorType.kBrushless);
        m_armSparkMax = new SparkMax(AlgaeConstants.kArmCanId, MotorType.kBrushless);
        m_armEncoder = m_armSparkMax.getAbsoluteEncoder();
        m_intakePIDController = m_intakeSparkMax.getClosedLoopController();
        m_armPIDController = m_armSparkMax.getClosedLoopController();

        m_intakeSparkMax.configure(Configs.AlgaeSubsystem.algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armSparkMax.configure(Configs.AlgaeSubsystem.algaeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
    }

    public void RunIntake(double speed) {
        m_intakeSparkMax.set(speed);
    }

    public void ArmToPosition(int goalPosition) {
        SmartDashboard.putNumber("Goal Arm Position", goalPosition);
        m_armPIDController.setReference(AlgaeConstants.algaePos[goalPosition], SparkMax.ControlType.kPosition);
    }
}
