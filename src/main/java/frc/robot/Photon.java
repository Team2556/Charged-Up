package frc.robot;

/*Photonvision subsystem- takes input from Limelight and gives the needed X-motion of the robot to 
line up with the seen Apriltag*/

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Photon extends SubsystemBase
{
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(29); //height of Limelight
    
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); //angle of camera
    //distance between the camera and target
    final double GOAL_RANGE_METERS = Units.inchesToMeters(16); 
    PhotonCamera camera = new PhotonCamera("photonvision");
    
    PhotonPipelineResult result = camera.getLatestResult();
    //AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
   // there are some errors with this ^
            //gets the list of targets seen
            PhotonTargetSortMode sortMode = PhotonTargetSortMode.Centermost;
            PhotonTargetSortMode photonTargetSortMode;
            List<PhotonTrackedTarget> targets = result.getTargets();
            //gets the best target
            PhotonTrackedTarget target = result.getBestTarget();
            //gets the id of the best target
            public int targetID = target.getFiducialId();
            //double yaw = target.getYaw();
            //double pitch = target.getPitch();
            //double area = target.getArea();
            //double skew = target.getSkew();
            // Transform2d pose = target.getCameraToTarget();
            // List<TargetCorner> corners = target.getCorners();
            
            public void Tag(){ //Calculates needed X-distance between robot and Apriltag
                double range = 0; //Linear distance between robot and Apriltag
                double movement = 0; //Stores the distance robot needs to move in inches
                if(targetID == 1){
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                
                else if(targetID == 2){
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                else if(targetID == 3){
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                else if(targetID == 4){
                //loading
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(24.38);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                else if(targetID == 5){
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                else if(targetID == 6){
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                else if(targetID == 7){
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                    SmartDashboard.putNumber("range", range);
                    
                }
                else if(targetID == 8){
                //loading
                    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(24.38);
                    range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getYaw()));
                            SmartDashboard.putNumber("range", range);
                            
                }
                range = range*39.36;
                movement = Math.sqrt((range * 2) - 256);
                 //move to the side
                 SmartDashboard.putNumber("movement", movement);
                 

            }

            public String test() {
                System.out.println("test");
                return "hi";
            }
            //outputs the tracked id to smart dashboard
                 private void updateSmartDashboard() {
                SmartDashboard.putNumber("targetID", targetID);
                
                System.out.println(targetID);
                System.out.println("Hi");
            }
                public void periodic() {
                    updateSmartDashboard();
                }
                public void targetID() {
                }
            
}