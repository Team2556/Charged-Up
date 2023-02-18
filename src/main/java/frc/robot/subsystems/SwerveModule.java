package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.CtreUtils;
import frc.robot.utility.RevUtils;

import static frc.robot.Constants.Swerve.Module.*;
import static frc.robot.Constants.Swerve.kMaxSpeedMetersPerSecond;

public class SwerveModule extends SubsystemBase {
    private final int POS_SLOT = 0;
    private final int VEL_SLOT = 1;

    int m_moduleNumber;
    CANSparkMax m_turnMotor;
    WPI_TalonFX m_driveMotor;
    private SparkMaxPIDController m_turnController;
    private final RelativeEncoder m_turnEncoder;
    DutyCycleEncoder m_angleEncoder;
    double m_angleOffset;
    double m_currentAngle;
    double m_lastAngle;

    Pose2d m_pose;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    ksDriveVoltSecondsPerMeter,
                    kaDriveVoltSecondsSquaredPerMeter,
                    kvDriveVoltSecondsSquaredPerMeter);

    private final ProfiledPIDController m_turningPIDController
            = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));

    public SwerveModule(
            int moduleNumber,
            CANSparkMax turnMotor,
            WPI_TalonFX driveMotor,
            DutyCycleEncoder angleEncoder,
            double angleOffset) {
        m_moduleNumber = moduleNumber;
        m_turnMotor = turnMotor;
        m_driveMotor = driveMotor;
        m_angleOffset = angleOffset;
        

        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());

        m_turnMotor.restoreFactoryDefaults();
        RevUtils.setTurnMotorConfig(m_turnMotor);
        m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_angleEncoder = angleEncoder;

        m_turnEncoder = m_turnMotor.getEncoder();
        m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees);
        m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60);

        m_turnController = m_turnMotor.getPIDController();

//        resetAngleToAbsolute();
    }

    public int getModuleNumber() {
        return m_moduleNumber;
    }

    public void resetAngleToAbsolute() {
        double angle = m_angleOffset - pwmToDegrees(m_angleEncoder.getAbsolutePosition());
        m_turnEncoder.setPosition(angle);
    }

    public double getHeadingDegrees() {
        return m_currentAngle;
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    public double getDriveMeters() {
        return m_driveMotor.getSelectedSensorPosition() * kDriveDistancePerPulse;
    }
    public double getDriveMetersPerSecond() {
        return m_driveMotor.getSelectedSensorVelocity() * kDriveDistancePerPulse * 10;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

        double velocity = desiredState.speedMetersPerSecond / (kDriveDistancePerPulse * 10);
        m_driveMotor.set(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                feedforward.calculate(desiredState.speedMetersPerSecond));

        double angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                        ? m_lastAngle
                        : desiredState.angle.getDegrees() - m_angleOffset; // Prevent rotating module if speed is less than 1%. Prevents Jittering.
        m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition, POS_SLOT);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
    }
    public void setModulePose(Pose2d pose) {
        m_pose = pose;
    }

    public Pose2d getModulePose() {
        return m_pose;
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber(
                "module " + m_moduleNumber + " heading", getState().angle.getDegrees());
        SmartDashboard.putNumber(
                "module " + m_moduleNumber + " CANCoder reading", pwmToDegrees(m_angleEncoder.getAbsolutePosition()));
    }

    private double pwmToDegrees(double signal) {
        return signal * 360.0;
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
