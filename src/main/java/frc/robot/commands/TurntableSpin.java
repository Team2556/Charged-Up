// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.Turntable;

public class TurntableSpin extends CommandBase {
  /** Creates a new Turntablespin. */

  private final Turntable m_turntableSubsystem;
  private final BButton m_BButton;

  public TurntableSpin(Turntable turntableSubsystem, Button BButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turntableSubsystem = turntableSubsystem;
    m_BButton = BButton;
    addRequirements(turntableSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
