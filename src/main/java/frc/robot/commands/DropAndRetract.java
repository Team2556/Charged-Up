// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants;

public class DropAndRetract extends CommandBase {
  /** Creates a new DropAndRetract. */
  ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
  boolean check = false;
  public DropAndRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clawSubsystem.clawOpenAction();
    if (m_armSubsystem.getLimitSwitch()) {
      m_armSubsystem.setExtensionMotor(-0.35);
      check = false;
    }
    else {
      m_armSubsystem.resetExtensionMotor();
      m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
      m_armSubsystem.setExtensionMotor(0.0);
      check = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
}
