// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Photon;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonCommand extends CommandBase {
  /** Creates a new PhotonCommand. */
  private Photon m_photonSubsystem;
  private Trigger m_x;
  Photon movementObj = new Photon();
  SwerveSubsystem moveObj;

  public PhotonCommand(Photon photonSubsystem, Trigger x, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    moveObj = swerve;
    m_photonSubsystem = photonSubsystem;
    m_x = x;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    double PhotonStrafe = movementObj.movement;
    double PhotonThrottle = 0;
    double PhotonRotation = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    moveObj.drive(PhotonThrottle*0.8, PhotonStrafe, PhotonRotation, true, true);
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
