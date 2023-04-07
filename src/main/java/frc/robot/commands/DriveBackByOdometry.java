// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;


public class DriveBackByOdometry extends CommandBase {
  /** Creates a new DriveForward. */
  SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();
  ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  
  Timer timer = new Timer();
  double m_distance;
  Boolean m_ArmDown;

  private boolean check = false;
  public DriveBackByOdometry(double distance, Boolean ArmDown) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem, m_armSubsystem);
    m_distance = distance;
    m_ArmDown = ArmDown;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    check = false;
    SmartDashboard.putString("where we b", "DriveFor");
    m_armSubsystem.setArmPosition(Constants.ArmPosition.GRAB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_ArmDown && m_swerveSubsystem.getOdometry().getPoseMeters().getX() > 0)
    m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
    if (m_swerveSubsystem.getOdometry().getPoseMeters().getX() < m_distance) {
      m_swerveSubsystem.drive(0.4,0,0,true);
      // m_swerveSubsystem.autoBalance();
      check = false;
    } else {
     m_swerveSubsystem.drive(0, 0, 0, true);
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
