// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.IntakeCoralCommandClearJam;
import frc.robot.commands.IsElevatorAtSetpointCommand;
import frc.robot.commands.OutputCoralCommand;
import frc.robot.commands.PushAgainstElement;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a 
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} 
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final LightsSubsystem m_lights = LightsSubsystem.getInstance();
    public Limelight m_limelight = Limelight.getInstance();
    public final ManipulatorSubsystem m_manipulator = ManipulatorSubsystem.getInstance();
    public final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getInstance();
    public final DriveSubsystem m_robotDrive = DriveSubsystem.getInstance();
    public final AlgaeSubsystem m_algaeSubsystem = AlgaeSubsystem.getInstance();
    private final ClimberSubsystem m_climberSubsystem = ClimberSubsystem.getInstance();
    public final PowerDistribution m_pdp = new PowerDistribution(9, ModuleType.kRev);
    
    // The driver controllers
    Joystick m_leftDriverController = new Joystick(OIConstants.kLeftDriverControllerPort);
    Joystick m_rightDriverController = new Joystick(OIConstants.kRightDriverControllerPort);

    // The operator controllers
    GenericHID m_operatorBoard = new GenericHID(OIConstants.kOperatorControllerPort1);

    public static boolean fieldOriented = false;
    public double speedMultiplier = OIConstants.kSpeedMultiplierDefault;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //register named commands
        NamedCommands.registerCommand("Level 0", new InstantCommand(
            ()-> m_elevatorSubsystem.ElevatorToSetpoint(0)));

        NamedCommands.registerCommand("Level 4", new SequentialCommandGroup(
            new InstantCommand(()-> m_elevatorSubsystem.ElevatorToSetpoint(4), m_elevatorSubsystem),
            new IsElevatorAtSetpointCommand(4)));

        NamedCommands.registerCommand("Intake", new IntakeCoralCommandClearJam(ManipulatorConstants.kManipulatorHighSpeed));

        NamedCommands.registerCommand("DeliverCoral", new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed));

        NamedCommands.registerCommand("Arm to 0", new InstantCommand(
            () -> m_algaeSubsystem.ArmToPosition(0)));

        NamedCommands.registerCommand("AlgaeFloorPickup", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0)),
            new InstantCommand(() -> m_algaeSubsystem.SetFloorPickup()),
            new InstantCommand(() -> m_algaeSubsystem.IntakeAlgae()),
            new InstantCommand(() -> m_algaeSubsystem.ArmToPosition(2))));

        NamedCommands.registerCommand("AlgaeReefPickupOne", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(3)),
            new IsElevatorAtSetpointCommand(3),
            new InstantCommand(() -> m_algaeSubsystem.SetReefPickup()),
            new InstantCommand(() -> m_algaeSubsystem.IntakeAlgae()),
            new InstantCommand(() -> m_algaeSubsystem.ArmToPosition(1))));

        NamedCommands.registerCommand("AlgaeReefPickupTwo", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(5)),
            new IsElevatorAtSetpointCommand(5),
            new InstantCommand(() -> m_algaeSubsystem.SetReefPickup()),
            new InstantCommand(() -> m_algaeSubsystem.IntakeAlgae()),
            new InstantCommand(() -> m_algaeSubsystem.ArmToPosition(1))));

        NamedCommands.registerCommand("AutoDeliverLevelOne", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(1)),
            new IsElevatorAtSetpointCommand(1),
            new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("AutoDeliverLevelTwo", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(2)),
            new IsElevatorAtSetpointCommand(2),
            new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("AutoDeliverLevelThree", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(3)),
            new IsElevatorAtSetpointCommand(3),
            new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("AutoDeliverLevelFour", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(4)),
            new IsElevatorAtSetpointCommand(4),
            new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("IntakeCoral", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0)),
            new IntakeCoralCommand(ManipulatorConstants.kManipulatorHighSpeed)));

        NamedCommands.registerCommand("DeliverCoral", new SequentialCommandGroup(
            new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("ElevatorToZero", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(7)),
            new IsElevatorAtSetpointCommand(7),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("Align Left Reef Branch", new SequentialCommandGroup(
            new AlignToReefTagRelative(false),
            new PushAgainstElement(.25, 0.25),
            new InstantCommand(() -> DataLogManager.log("Left reef alignment completed"))
                    ));

        NamedCommands.registerCommand("Align Right Reef Branch", new SequentialCommandGroup(
            new AlignToReefTagRelative(true),
            new PushAgainstElement(0.25, 0.25),
            new InstantCommand(() -> DataLogManager.log("Right reef alignment completed"))
            ));

        NamedCommands.registerCommand("Prepare to climb", new SequentialCommandGroup(
            new InstantCommand(() -> m_lights.setLEDs(LightsConstants.STROBE_RED)),
            new InstantCommand(() -> m_climberSubsystem.WinchRatchetSetPosition(1)),
            new InstantCommand(() -> m_climberSubsystem.ClimberIntakeToSetpoint(1)),
            new InstantCommand(() -> m_climberSubsystem.ClimberWinchToSetpoint(1)),
            new InstantCommand(() -> DataLogManager.log("Climber preparation Completed"))
            )
        );

        NamedCommands.registerCommand("Climb", new SequentialCommandGroup(
            new InstantCommand(() -> m_climberSubsystem.WinchRatchetSetPosition(0)),
            new InstantCommand(() -> m_climberSubsystem.ClimberWinchToSetpoint(0)),
            new InstantCommand(() -> DataLogManager.log("Climbing"))
            )
        );

        //Take up any space between the robot and the human player station
        NamedCommands.registerCommand("Go To Feeder Station", 
            new PushAgainstElement(-0.5, 0.5) );

        configureButtonBindings();
        m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftDriverController.getRawAxis(1) * speedMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftDriverController.getRawAxis(0) * speedMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_rightDriverController.getRawAxis(0) * speedMultiplier, OIConstants.kDriveDeadband) * OIConstants.kRotateScale,
                fieldOriented, true),
            m_robotDrive));
        autoChooser = AutoBuilder.buildAutoChooser(); 
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {

        //Driver Controls

       new JoystickButton(m_leftDriverController,OIConstants.kJS_BB)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive).alongWith(
                    new InstantCommand(() -> DataLogManager.log("L Driver JS_BB (SetX) pressed"))));

        new JoystickButton(m_rightDriverController, OIConstants.kJS_RB).debounce(0.1)  //Gyro reset
            .whileTrue(new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive).alongWith(
                    new InstantCommand(() -> DataLogManager.log("R Driver JS_RB (Zero gyro) pressed")))); 

        new JoystickButton(m_rightDriverController, OIConstants.kJS_LB)  //Field oriented toggle
            .whileTrue(new InstantCommand(
                () -> toggleFieldOriented()).alongWith(
                    new InstantCommand(() -> DataLogManager.log("R Driver JS_LB (Field Oriented) pressed"))));

        new JoystickButton(m_leftDriverController, OIConstants.kJS_Trigger)  
            .whileTrue(new InstantCommand(  //Precise Driving Mode set
                () -> speedMultiplier=OIConstants.kSpeedMultiplierPrecise).alongWith(
                    new InstantCommand(() -> DataLogManager.log("L Driver Trigger (Precision Driving) pressed")))) 
            .whileFalse(new InstantCommand( //Precise Driving Mode clear
                () -> speedMultiplier=OIConstants.kSpeedMultiplierDefault));

        // new JoystickButton(m_leftDriverController, OIConstants.kJS_LB)
        //     .whileTrue(new DriveWithAprilTagCommandOffset(
        //         m_leftDriverController, m_rightDriverController, true).alongWith(
        //             new InstantCommand(() -> DataLogManager.log("L Driver JS_LB (DriveAprilTagL) pressed"))));

        // new JoystickButton(m_leftDriverController, OIConstants.kJS_RB)
        //     .whileTrue(new DriveWithAprilTagCommandOffset(
        //         m_leftDriverController, m_rightDriverController, false).alongWith(
        //             new InstantCommand(() -> DataLogManager.log("L Driver JS_RB (DriveAprilTagR) pressed"))));

        new JoystickButton(m_leftDriverController, OIConstants.kJS_LB)
            .onTrue(NamedCommands.getCommand("Align Left Reef Branch").alongWith(
                new InstantCommand(() -> DataLogManager.log("R Driver LBLB (Align L Reef) pressed"))));

        new JoystickButton(m_leftDriverController, OIConstants.kJS_RB)
            .onTrue(NamedCommands.getCommand("Align Right Reef Branch").alongWith(
                new InstantCommand(() -> DataLogManager.log("R Driver RBRB (Align R Reef) pressed"))));

        //Operator Controls

        new JoystickButton(m_operatorBoard, 6) 
             .onTrue(new InstantCommand(
                 () -> m_algaeSubsystem.IntakeAlgae()).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB6 (IntakeAlgae) pressed"))));    

        new JoystickButton(m_operatorBoard, 7)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.DeliverAlgae()).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB7 (DeliverAlgae) pressed"))));

        new JoystickButton(m_operatorBoard, 5)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.StopIntake()).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB5 (StopIntake) pressed"))));

        new JoystickButton(m_operatorBoard, 13)
            .onChange(NamedCommands.getCommand("Prepare to climb").alongWith(
                new InstantCommand(() -> DataLogManager.log("BB13 (Prepare to Climb) engaged"))));
                /*.onChange(new InstantCommand(
                () -> m_algaeSubsystem.TogglePickup()).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB13 (AlgaeTogglePickup) pressed"))));*/

        new JoystickButton(m_operatorBoard, 21)
            .onTrue(NamedCommands.getCommand("Climb").alongWith(
                new InstantCommand(() -> DataLogManager.log("BB21 (Climbing"))));
                /*() -> m_manipulator.RunManipulator(ManipulatorConstants.kManipulatorHighSpeed), m_manipulator).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB21 (DischargeCoral) pressed"))));*/
        

        new JoystickButton(m_operatorBoard, 3)
            .onTrue(NamedCommands.getCommand("AlgaeReefPickupOne").alongWith(
                new InstantCommand(() -> DataLogManager.log("BB3 (AlgaeReefPickupOne) pressed"))));

        new JoystickButton(m_operatorBoard, 4)
            .onTrue(NamedCommands.getCommand("AlgaeReefPickupTwo").alongWith(
                new InstantCommand(() -> DataLogManager.log("BB4 (AlgaeReefPickupTwo) pressed"))));

        new JoystickButton(m_operatorBoard, 2)
             .onTrue(NamedCommands.getCommand("AlgaeFloorPickup").alongWith(
                new InstantCommand(() -> DataLogManager.log("BB2 (AlgaeFloorPickup) pressed"))));

        new JoystickButton(m_operatorBoard, 1)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.ArmToPosition(0)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB1 (Algae ArmToPos0) pressed")))); 

        new JoystickButton(m_operatorBoard, 11)
            .onTrue(new OutputCoralCommand(ManipulatorConstants.kManipulatorHighSpeed).alongWith(
                new InstantCommand(() -> DataLogManager.log("BB11 (OutputCoral) pressed"))));

        new JoystickButton(m_operatorBoard, 15)
            .onTrue(new IntakeCoralCommandClearJam(ManipulatorConstants.kManipulatorMedSpeed).alongWith(
                new InstantCommand(() -> DataLogManager.log("BB15 (IntakeCoral) pressed"))));

        
        new JoystickButton(m_operatorBoard, 19)
            .onTrue(new InstantCommand(
                () -> m_manipulator.StopManipulator(), m_manipulator).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB19 (StopManipulator) pressed"))));

        new JoystickButton(m_operatorBoard, 22)
            .onTrue(new InstantCommand(
                () -> m_manipulator.RunManipulator(-ManipulatorConstants.kManipulatorHighSpeed)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB22 (ReverseManipulator) pressed")))); 

        new JoystickButton(m_operatorBoard, 12)
            .onChange(new InstantCommand(
                () -> m_elevatorSubsystem.overrideSensor()).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB12 (ElevatorOverrideSensor) pressed")))); 

        new JoystickButton(m_operatorBoard, 18)
            .onTrue(NamedCommands.getCommand("ElevatorToZero").alongWith(
                new InstantCommand(() -> DataLogManager.log("BB18 (ElevatorToZero) pressed"))));

        new JoystickButton(m_operatorBoard, 17)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(1)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB17 (ElevatorToOne) pressed"))));

        new JoystickButton(m_operatorBoard, 16)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(2)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB16 (ElevatorToTwo) pressed"))));

        new JoystickButton(m_operatorBoard, 14)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(3)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB14 (ElevatorToThree) pressed"))));

        new JoystickButton(m_operatorBoard, 8)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(4)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB8 (ElevatorToFour) pressed"))));

        new JoystickButton(m_operatorBoard, 20)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(6)).alongWith(
                    new InstantCommand(() -> DataLogManager.log("BB20 (ElevatorToSix) pressed"))));
        }

    private void toggleFieldOriented () {
        fieldOriented = !fieldOriented;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
