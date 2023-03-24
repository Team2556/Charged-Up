// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.ExtensionPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants;

public class ArmRetractAndPrep extends CommandBase {
  ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();

  private boolean check = false, firstAutoLoop = true;
  /** Creates a new ArmRetractAndPrep. */
  public ArmRetractAndPrep() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    check = false;
    firstAutoLoop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(firstAutoLoop) {
      m_clawSubsystem.clawCloseAction();
      m_armSubsystem.setExtensionPosition(ExtensionPosition.RETRACT);
      firstAutoLoop = false;
      check = false;
    }

    if(almostEqual(m_armSubsystem.getExtensionPosition().getPosition(), m_armSubsystem.getExtensionEncoderPosition(), 3)) {
      m_armSubsystem.setArmPosition(ArmPosition.START);
    }

    if(almostEqual(m_armSubsystem.getArmPosition().getPosition(), m_armSubsystem.getArmEncoderPosition(), 3)
        && m_armSubsystem.getArmPosition().equals(Constants.ArmPosition.START)) {
      check = true;
    }

    m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
    m_armSubsystem.setExtensionMotor(m_armSubsystem.getExtensionPosition().getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
  private boolean almostEqual(double a, double b, double eps){
    return Math.abs(a-b) < eps;
}
}
