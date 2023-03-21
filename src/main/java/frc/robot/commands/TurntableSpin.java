// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableSpin extends CommandBase {
  /** Creates a new TurntableSpin. */
  private final TurntableSubsystem m_turntableSubsystem;
  private Trigger m_b;
 // private final BButton m_BButton;

  public TurntableSpin(TurntableSubsystem turntableSubsystem, Trigger b){//, Button BButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turntableSubsystem = turntableSubsystem;
    m_b = b;
   // m_BButton = BButton;
    addRequirements(turntableSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turntableSubsystem.TurntableReset();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_turntableSubsystem.TurntableOps();)
    
    m_turntableSubsystem.TurntableSpin(m_turntableSubsystem.ToggleTurntable(m_b));
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
