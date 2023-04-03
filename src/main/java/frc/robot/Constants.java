// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static double armEncoderOffset = 140.1;
    public static double kArmP = 0.25;
    public static double kArmI = 0.0;
    public static double kArmD = 0.0;
    public static int kTimeoutMs = 30;

    public enum ArmPosition {
        START(-109.0),
        GRAB(-109.0),
        INTAKE(-90.0 - 4.0),
        CONE_LOW(-75.0),
        CONE_MEDIUM(-20.0),
        CONE_HIGH(5.0),
        CUBE_LOW(-75.0),
        CUBE_MEDIUM(-25.0),
        CUBE_HIGH(0.0),
        SLIDE_GRAB(0.0);

        private final double position;

        ArmPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum ExtensionPosition {
        RESET(0.0),
        GRAB_CONE(105.0),
        GRAB_CUBE(60.0),
        PLACE_CONE(60.0),
        EXTEND(120.0),
        RETRACT(0.0);

        private final double position;

        ExtensionPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static class Ports {
        // Pneumatic Hub Ports
        public static final int pneumaticHubCANID = 9;
        public static final int intakeSolenoidForward = 4;
        public static final int intakeSolenoidReverse = 5;
        public static final int grabSolenoidForward = 6;
        public static final int grabSolenoidReverse = 7;
        // Motor Ports
        public static final int intakeMotorPort = 10;
        public static final int armMotorPort = 13;
        public static final int extensionMotorPort = 14;
        public static final int clawNeoPort = 15;
        // Digital Input Ports
        public static final int armAbsoluteEncoderPort = 0;
        public static final int extensionLimitSwitchPort = 4;
    }

    public static class OperatorConstants {
        // Controller port for driving
        public static final int kDriverControllerPort = 0;
        // Controller port for mechanisms (like arm/claw)
        public static final int kMechanismControllerPort = 1;
    }

    public static final class Swerve {
        public static final double kTrackWidth = Units.inchesToMeters(30.0);
        public static final double kWheelBase = Units.inchesToMeters(30.0);

        public static final Translation2d[] kModuleTranslations = {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2)
        };

        public static final double frontLeftCANCoderOffset = 0.0;
        public static final double frontRightCANCoderOffset = 0.0;
        public static final double backLeftCANCoderOffset = 0.0;
        public static final double backRightCANCoderOffset = 0.0;

        public static final SwerveDriveKinematics kSwerveKinematics =
                new SwerveDriveKinematics(kModuleTranslations);

        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(16);
        public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
        public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

        public static final double kP_X = 0.1;
        public static final double kD_X = 0;
        public static final double kP_Y = 0.1;
        public static final double kD_Y = 0;
        public static final double kP_Theta = 8;
        public static final double kD_Theta = 0;

        public static final double kPathingX_kP = 0.1;
        public static final double kPathingX_kI = 0;
        public static final double kPathingX_kD = 0;
        public static final double kPathingY_kP = 0.1;
        public static final double kPathingY_kI = 0;
        public static final double kPathingY_kD = 0;
        public static final double kPathingTheta_kP = 3;
        public static final double kPathingTheta_kI = 0;
        public static final double kPathingTheta_kD = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }

        public static final class Module {
            public static final double kDriveMotorGearRatio = 7.0;
            public static final double kTurningMotorGearRatio = 13.333;
            public static final double kWheelDiameterMeters = Units.inchesToMeters(3.89);
            public static final int kNeoCPR = 42;
            public static final int kFalconEncoderCPR = 2048;

            public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
            public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);

            public static final double kDriveDistancePerPulse =
                    (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveMotorGearRatio);

            public static final double kDriveRevToMeters =
                    ((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio);
            public static final double kDriveRpmToMetersPerSecond =
                    kDriveRevToMeters / 60.0;
            public static final double kTurnRotationsToDegrees =
                    360.0 / kTurningMotorGearRatio;

            public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
            public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
            public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

            public static final double kvTurnVoltSecondsPerRadian = 1.47;
            public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348;
        }

        public static final class Ports {
            public static final int frontLeftDriveMotor = 1;
            public static final int frontLeftTurnMotor = 2;
            public static final int backLeftDriveMotor = 3;
            public static final int backLeftTurnMotor = 4;
            public static final int backRightDriveMotor = 5;
            public static final int backRightTurnMotor = 6;
            public static final int frontRightDriveMotor = 7;
            public static final int frontRightTurnMotor = 8;
        }
    }
}