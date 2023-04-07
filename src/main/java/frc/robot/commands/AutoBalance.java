// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();
  private Timer timer = new Timer();
  private static BalanceState state = BalanceState.DRIVE_OVER;
  private double firstX;
  private double currentX;
  private double pitch;

  public enum BalanceState {
    DRIVE_OVER,
    BACK_UP,
    DRIVE_UP,
    START_PITCH,
    FINAL_CHECK
  }

  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    m_swerveSubsystem.gyro.reset();
    m_swerveSubsystem.setIsFieldRelative(false);
    state = BalanceState.DRIVE_OVER;
  }

  // Called every time the scheduler runs while the command is scheduled.z
  @Override
  public void execute() {
    // m_swerveSubsystem.drive(-0.5, 0, 0, true);
    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putString("case", state.toString());
    switch (state) {
      case DRIVE_OVER:
        if (timer.get() < 3.75) {
          m_swerveSubsystem.drive(0.3, 0, 0, true);
        } else {
          m_swerveSubsystem.drive(0, 0, 0, true);
          state = BalanceState.BACK_UP;
        }
        break;
      case BACK_UP:
        if (Math.abs(m_swerveSubsystem.gyro.getPitch()) <= 18.0 ) {
          m_swerveSubsystem.drive(-0.2, 0, 0, true);
        } else {
          m_swerveSubsystem.drive(0, 0, 0, true);
          state = BalanceState.DRIVE_UP;
        }
        break;
      case DRIVE_UP:
        if (Math.abs(m_swerveSubsystem.gyro.getPitch()) >= 18.0 ) {
          m_swerveSubsystem.drive(-0.125, 0, 0, true);
        } else {
          m_swerveSubsystem.drive(0, 0, 0, true);
          firstX = m_swerveSubsystem.getOdometry().getPoseMeters().getX() - 0.08;
          pitch = m_swerveSubsystem.gyro.getPitch();
          timer.reset();
          state = BalanceState.START_PITCH;
        }
        break;
      case START_PITCH:
      if (Math.abs(m_swerveSubsystem.gyro.getPitch()) >= 17.0 ) {
          m_swerveSubsystem.drive(0.35, 0, 0, true);
        } else {
          m_swerveSubsystem.drive(0, 0, 0, true);
          //firstX = m_swerveSubsystem.getOdometry().getPoseMeters().getX();
          state = BalanceState.FINAL_CHECK;
        }
        break;
      case FINAL_CHECK:

         if (timer.get() < .75)  {
          m_swerveSubsystem.drive(.35, 0, 0, true);
        } else {
          m_swerveSubsystem.drive(0, 0, 0, true);
        }
        // if (m_swerveSubsystem.gyro.getPitch() >= 0.5) {
        //   m_swerveSubsystem.drive(-0.1, 0, 0, true);
        // } else if (m_swerveSubsystem.gyro.getPitch() <= -0.5) {
        //   m_swerveSubsystem.drive(0.1, 0, 0, true);
        // } else {
        //   m_swerveSubsystem.drive(0, 0, 0, true);
        // }
        break;
    }
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
