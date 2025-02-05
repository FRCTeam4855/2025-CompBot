package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Configs;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem {

    public final SparkMax rightSparkMax;
    public final SparkMax leftSparkMax;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public final SparkAbsoluteEncoder rightAbsoluteEncoder;
    public final SparkAbsoluteEncoder leftAbsoluteEncoder;

    public ManipulatorSubsystem() {

        rightSparkMax = new SparkMax(ManipulatorConstants.kRightManipulatorCanId, MotorType.kBrushless);
        leftSparkMax = new SparkMax(ManipulatorConstants.kLeftManipulatorCanId, MotorType.kBrushless);
        rightPIDController = rightSparkMax.getClosedLoopController();
        leftPIDController = leftSparkMax.getClosedLoopController();
        rightAbsoluteEncoder = rightSparkMax.getAbsoluteEncoder();
        leftAbsoluteEncoder = leftSparkMax.getAbsoluteEncoder();

        rightSparkMax.configure(Configs.ManipulatorSubsystem.rightManipulatorConfig, 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftSparkMax.configure(Configs.ManipulatorSubsystem.leftManipulatorConfig, 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}