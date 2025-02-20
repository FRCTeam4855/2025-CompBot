package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorSubsystem extends SubsystemBase {

    public final ManipulatorSubsystem m_manipulator;
    public final SparkFlex rightSpark;
    public final SparkFlex leftSpark;
    public final RelativeEncoder rightEncoder;
    public final RelativeEncoder leftEncoder;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public boolean sensorOverride = false;

    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_curState = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_goalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();
    private double dt;
    private double curTime;


    public ElevatorSubsystem(ManipulatorSubsystem m_manipulator) {
        this.m_manipulator = m_manipulator;

        rightSpark = new SparkFlex(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
        leftSpark = new SparkFlex(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
        rightEncoder = rightSpark.getEncoder();
        leftEncoder = leftSpark.getEncoder();
        rightPIDController = rightSpark.getClosedLoopController();
        leftPIDController = leftSpark.getClosedLoopController();

        rightSpark.configure(Configs.ElevatorSubsystem.rightElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        leftSpark.configure(Configs.ElevatorSubsystem.leftElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.kMaxVelocity,
                Constants.ElevatorConstants.kMaxAcceleration));
    }

    @Override
    public void periodic() {
        curTime = Timer.getFPGATimestamp();
        SmartDashboard.putBoolean("Elevator Clear", m_manipulator.isElevatorClear());
        SmartDashboard.putNumber("Right Elevator Pos", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Elevator Pos", leftEncoder.getPosition());
    }

    public void raiseElevator(int goalSetpoint) {
        if (m_manipulator.isElevatorClear() || sensorOverride) {
            m_goalState.position = ElevatorConstants.elevatorPos[goalSetpoint];
            dt = curTime - prevUpdateTime;
            m_curState = m_profile.calculate(dt, m_curState, m_goalState);
            rightPIDController.setReference(m_curState.position, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kElevatorConstantsGravityFF, ArbFFUnits.kVoltage);
            prevUpdateTime = curTime;
        } else {
            System.out.println("! ELEVATOR NOT CLEAR !");
        }
    }

    public void overrideSensor() {
        sensorOverride =! sensorOverride;
    }
}