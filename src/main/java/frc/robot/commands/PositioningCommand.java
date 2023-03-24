// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.ExtensionPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PositioningCommand extends CommandBase {
  /** Creates a new ArmPositioningCommand. */
  ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
  SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();
  ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
  Timer timer = new Timer();

  private boolean check = false;

  private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath(
    "StartCone", Constants.Swerve.kMaxSpeedMetersPerSecond,3);

  // PID controller for movement in the X direction
  private PIDController mXController = new PIDController(
    Constants.Swerve.kPathingX_kP, Constants.Swerve.kPathingX_kI, Constants.Swerve.kPathingX_kD);
  // PID controller for movement in the Y direction
  private PIDController mYController = new PIDController(
    Constants.Swerve.kPathingY_kP, Constants.Swerve.kPathingY_kI, Constants.Swerve.kPathingY_kD);
 // PID controller for robot heading
 private PIDController mThetaController = new PIDController(
    Constants.Swerve.kPathingTheta_kP, Constants.Swerve.kPathingTheta_kI, 
    Constants.Swerve.kPathingTheta_kD);

  PPSwerveControllerCommand path = new PPSwerveControllerCommand(
    mTrajectory, 
    m_swerveSubsystem.poseSupplier, 
    Constants.Swerve.kSwerveKinematics, 
    mXController, 
    mYController, 
    mThetaController, 
    m_swerveSubsystem::setModuleStates, 
    m_swerveSubsystem
  );
  public PositioningCommand() {
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
    m_armSubsystem.setArmPosition(ArmPosition.CONE_HIGH);
    m_armSubsystem.setExtensionPosition(ExtensionPosition.EXTEND);
    m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
    m_armSubsystem.setExtensionMotor(m_armSubsystem.getExtensionPosition().getPosition());
    path.andThen(() -> m_swerveSubsystem.drive(0, 0, 0, true));
    m_clawSubsystem.clawOpenAction();
    check = true;
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
