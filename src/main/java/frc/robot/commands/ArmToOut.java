// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ExtensionPosition;

public class ArmToOut extends CommandBase {
  /** Creates a new ArmToOut. */
  ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
  private boolean check = false;
  private int ctr = 0;
  // private Timer timer;
  public ArmToOut() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.start();
    m_armSubsystem.setArmPosition(Constants.ArmPosition.CONE_HIGH);
    m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
    // m_armSubsystem.setExtensionPosition(ExtensionPosition.GRAB_CONE);
    check = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clawSubsystem.clawCloseAction();
    m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
    // m_armSubsystem.setExtensionMotor(0.25);
    SmartDashboard.putNumber("ext spd", m_armSubsystem.getExtMotorSpeed());
    m_armSubsystem.setExtensionPositionPID(180); //m_armSubsystem.getExtensionPosition().getPosition());
    if (m_armSubsystem.getExtensionEncoderPosition() <= 180.0) {
      m_armSubsystem.setExtensionMotor(0.25);
      ctr++;
      SmartDashboard.putString("is ext", "yes" + ctr);
    } else {
      m_armSubsystem.setExtensionMotor(0.0);
    }
    if (m_armSubsystem.getArmPosition() == Constants.ArmPosition.CONE_HIGH && m_armSubsystem.getExtensionEncoderPosition() >= 180) {
      check = true;
    } else {
      check = false;
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
