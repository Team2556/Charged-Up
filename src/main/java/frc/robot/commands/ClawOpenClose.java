// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;

public class ClawOpenClose extends CommandBase {
  /** Creates a new ClawOpenClose. */
  private final Claw m_clawSubsystem;
  private final Trigger m_lBumper;
  private final Trigger m_rBumper;

  public ClawOpenClose(Claw clawSubsystem, Trigger lBumper, Trigger rBumper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_clawSubsystem = clawSubsystem;
    m_lBumper = lBumper;
    m_rBumper = rBumper;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Claw.clawReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Claw.clawOpenButton(m_rBumper.getAsBoolean());
    Claw.clawCloseButton(m_lBumper.getAsBoolean());
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
