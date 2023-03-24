// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class GrabCommand extends CommandBase {
  /** Creates a new GrabCommand. */
  private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  private final ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
  private boolean check = false;
  private boolean firstAutoLoop = true;
  public GrabCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("auto test", "test");
    firstAutoLoop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(firstAutoLoop) {
      m_clawSubsystem.clawOpenAction();
      m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.GRAB);
      firstAutoLoop = false;
      check = false;
  }

  if(almostEqual(m_armSubsystem.getExtensionPosition().getPosition(), m_armSubsystem.getExtensionEncoderPosition(), 3)) {
      if(m_armSubsystem.getCones())
          m_clawSubsystem.clawCloseAction();
      m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
  }

  if(almostEqual(m_armSubsystem.getExtensionPosition().getPosition(), m_armSubsystem.getExtensionEncoderPosition(), 3)
  && m_armSubsystem.getExtensionPosition().equals(Constants.ExtensionPosition.RETRACT)) {
    check = true;
  }

  m_clawSubsystem.clawPullAction();
  m_armSubsystem.setExtensionPositionPID(m_armSubsystem.getExtensionPosition().getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setExtensionMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }

  private boolean almostEqual(double a, double b, double eps){
    return Math.abs(a-b) < eps;
  }
}
