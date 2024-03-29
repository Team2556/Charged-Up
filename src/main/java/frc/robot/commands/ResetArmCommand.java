// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.Constants;

public class ResetArmCommand extends CommandBase {
  /** Creates a new ResetCommand. */
  private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  private final CompressorSubsystem compressorSubsystem = CompressorSubsystem.getInstance();
  private boolean check = false;
  public ResetArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    compressorSubsystem.update();
    if (m_armSubsystem.getLimitSwitch())
      m_armSubsystem.setExtensionMotor(-0.25);
    else {
      m_armSubsystem.resetExtensionMotor();
      m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
      check = true;
    }
    SmartDashboard.putBoolean("first test", false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("end", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
}
