// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PhotonCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Photon;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  Photon photonSubsystem = new Photon();
  PhotonCommand run = new PhotonCommand(photonSubsystem, null, swerveSubsystem);
  //Photon photon = new Photon();

  public static boolean fieldRelativeDriving = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox1 =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands
    PhotonCamera camera = new PhotonCamera("photonvision");
    
    PhotonPipelineResult result = camera.getLatestResult();
            //gets the list of targets seen
           // List<PhotonTrackedTarget> targets = result.getTargets();
            //gets the best target
            PhotonTargetSortMode sortMode = PhotonTargetSortMode.Centermost; 
            //PhotonPipelineResult result2 = new PhotonPipelineResult();
            PhotonTrackedTarget target = result.getBestTarget();
            //PhotonTrackedTarget target1 = result.gets ;
            //gets the id of the best target
            int targetID = target.getFiducialId(); 
            //outputs the tracked id to smart dashboard
                SmartDashboard.putNumber("targetID", targetID);
                System.out.println(targetID);
                System.out.println("Hi");

    swerveSubsystem.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new SwerveDrive(
                    swerveSubsystem,
                    xbox1::getLeftY,
                    xbox1::getLeftX,
                    xbox1::getRightX,
                    fieldRelativeDriving));
    photonSubsystem.setDefaultCommand(
      new PhotonCommand(
        photonSubsystem,
        xbox1.y(), swerveSubsystem));}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new WaitCommand(0);
  }
}
