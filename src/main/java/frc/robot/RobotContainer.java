// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Map;

import static frc.robot.Constants.Swerve.kMaxSpeedMetersPerSecond;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final ClawSubsystem       clawSubsystem               = ClawSubsystem.getInstance();
    private final CompressorSubsystem compressorSubsystem         = CompressorSubsystem.getInstance();
    private final TurntableSubsystem  turntableSubsystem          = TurntableSubsystem.getInstance();
    private final SwerveSubsystem     swerveSubsystem             = SwerveSubsystem.getInstance();
    private final ArmSubsystem        armSubsystem                = ArmSubsystem.getInstance();
    private final IntakeSubsystem     intakeSubsystem             = IntakeSubsystem.getInstance();

    private final CommandXboxController xbox1 =
        new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final CommandXboxController xbox2 =
            new CommandXboxController(Constants.OperatorConstants.kMechanismControllerPort);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private GenericEntry kAutoStartDelaySeconds;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Autonomous selector options
        kAutoStartDelaySeconds = Shuffleboard.getTab("Live")
                .add("Auto Delay", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties((Map.of("Min", 0, "Max", 10, "Block increment", 1)))
                .getEntry();
        autoChooser.setDefaultOption("Regular Auto", new AutoGroup());
        autoChooser.addOption("Nothing", Commands.waitSeconds(5));
        autoChooser.addOption("Auto Balance", swerveSubsystem.autoBalance());
        // Configure default commands
        swerveSubsystem.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new SwerveDrive(
                    swerveSubsystem,
                    xbox1::getLeftY,
                    xbox1::getLeftX,
                    xbox1::getRightX,
                    xbox1.y())
        );

        intakeSubsystem.setDefaultCommand(
            new IntakeControl(
                    xbox1::getRightTriggerAxis,
                    xbox1::getLeftTriggerAxis,
                    xbox1.rightBumper(),
                    xbox1.leftBumper())
        );

        armSubsystem.setDefaultCommand(
                new ArmControl(
                        xbox2::getLeftY,
                        xbox2::getRightY)
        );

//        armSubsystem.setDefaultCommand(
//                new ArmTuner(
//                        xbox2::getLeftY)
//        );

        clawSubsystem.setDefaultCommand(
            new ClawControl(
                    xbox2.leftTrigger(),
                    xbox2.rightTrigger(),
                    xbox2.leftBumper(),
                    xbox2.rightBumper())
        );

        compressorSubsystem.setDefaultCommand(
            new CompressorCommand()
        );

        turntableSubsystem.setDefaultCommand(
            new TurntableSpin(
                    xbox2.start(),
                    xbox2.b()
            )
        );

        // Configure the trigger bindings
        configureBindings();
    }
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        //ToDo Do this in a better way
        xbox1.start().onTrue(new InstantCommand(() -> swerveSubsystem.setIsFieldRelative(!swerveSubsystem.getIsFieldRelative())));
        xbox1.a().onTrue(new InstantCommand(() -> SwerveDrive.setPrecisionMode(!SwerveDrive.getPrecisionMode())));
        xbox2.a().onTrue(new InstantCommand(() -> turntableSubsystem.setManualToggle(!turntableSubsystem.getManualToggle())));
        xbox2.y().onTrue(new InstantCommand(() -> armSubsystem.setCones(!armSubsystem.getCones())));
        xbox2.x().onTrue(new InstantCommand(() -> ArmControl.setArmState(ArmControl.ArmState.AUTO_PICKUP)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        return new AutoGroup();
    }
}

//swerveSubsystem.getTimer().reset();
        // return Commands.sequence(
        //         Commands.race(
        //                 Commands.sequence(
        //                         Commands.run(
        //                                 ()->swerveSubsystem.drive(0.5/kMaxSpeedMetersPerSecond,
        //                                         0,0,true),swerveSubsystem).until(()->swerveSubsystem.getTimer().get() > 5.0))));
//                new AutoPlace());
//                autoChooser.getSelected());
//                swerveSubsystem.autoBalance());
