// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class ArmToManual extends CommandBase {
  /** Creates a new ArmToManual. */
  private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  private final ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
  private boolean check = false;
  public ArmToManual() {
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
    ArmControl.setArmState(ArmControl.ArmState.MANUAL);
    check = true;
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
