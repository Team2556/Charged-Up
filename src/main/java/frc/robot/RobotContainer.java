// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmTuner;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.ClawOpenClose;
import frc.robot.commands.ClawPullPush;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.commands.CompressorCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurntableSubsystem;
import frc.robot.commands.TurntableSpin;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  Claw clawSubsystem = new Claw();
  Claw clawSpinSubsystem = new Claw();
  CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
  TurntableSubsystem turntableSubsystem = new TurntableSubsystem();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Arm armSubsystem = Arm.getInstance();
  private final Intake intakeSubsystem = Intake.getInstance();

  public static boolean fieldRelativeDriving = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox1 =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController xbox2 =
          new CommandXboxController(Constants.OperatorConstants.kMechanismControllerPort);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands

    swerveSubsystem.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
          new SwerveDrive(
                  swerveSubsystem,
                  xbox1::getLeftY,
                  xbox1::getLeftX,
                  xbox1::getRightX,
                  xbox1.a(),
                  fieldRelativeDriving));

    armSubsystem.setDefaultCommand(
            new ArmTuner(xbox2::getLeftY)
    );

    intakeSubsystem.setDefaultCommand(new IntakeControl(xbox2::getRightTriggerAxis, xbox2::getLeftTriggerAxis));

    // Configure the trigger bindings
    configureBindings();
    clawSubsystem.setDefaultCommand(
            new ClawOpenClose(
                    clawSubsystem,
                    xbox1.leftBumper(),
                    xbox1.rightBumper()));
    clawSpinSubsystem.setDefaultCommand(
            new ClawPullPush(
                    clawSpinSubsystem,
                    xbox1.x(),
                    xbox1.y())); 
     compressorSubsystem.setDefaultCommand(
            new CompressorCommand(
                    compressorSubsystem));
    turntableSubsystem.setDefaultCommand(
            new TurntableSpin(
                      turntableSubsystem,
                      xbox1.b()));//,
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
}
