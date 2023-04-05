// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForwardByOdometry extends CommandBase {
  /** Creates a new DriveForward. */
  SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();
  Timer timer = new Timer();
  private boolean check = false;
  public DriveForwardByOdometry() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    check = false;
    SmartDashboard.putString("where we b", "DriveFor");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_swerveSubsystem.getOdometry().getPoseMeters().getX() > -0.25) {
      m_swerveSubsystem.drive(-0.1,0,0,true);
      // m_swerveSubsystem.autoBalance();
      check = false;
    } else {
     m_swerveSubsystem.drive(0, 0, 0, true);
     new WaitCommand(0.5);
     check = true;
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
