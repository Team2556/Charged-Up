// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;

public class ClawPullPush extends CommandBase {
  private final ClawSubsystem m_clawSubsystemSpinSubsystem;
  private final Trigger m_x;
  private final Trigger m_y;

  public ClawPullPush(ClawSubsystem clawSubsystemSpinSubsystem, Trigger x, Trigger y) {
    m_clawSubsystemSpinSubsystem = clawSubsystemSpinSubsystem;
    m_x = x;
    m_y = y;
    addRequirements(m_clawSubsystemSpinSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clawSubsystemSpinSubsystem.clawButton(m_x.getAsBoolean(), m_y.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
