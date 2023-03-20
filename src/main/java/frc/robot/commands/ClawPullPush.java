// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;

public class ClawPullPush extends CommandBase {
  /** Creates a new ClawPullPush. */
  private Claw m_clawSpinSubsystem;
  private Trigger m_x;
  private Trigger m_y;

  public ClawPullPush(Claw clawSpinSubsystem, Trigger x, Trigger y) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_clawSpinSubsystem = clawSpinSubsystem;
    m_x = x;
    m_y = y;
    addRequirements(m_clawSpinSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Claw.clawButton(m_x.getAsBoolean(), m_y.getAsBoolean());
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
