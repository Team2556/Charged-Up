package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
    private static final ArmSubsystem instance = getInstance();
    private final WPI_TalonFX armMotor = new WPI_TalonFX(armMotorPort);
    private final CANSparkMax extension = new CANSparkMax(extensionMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(extensionLimitSwitchPort);
    public final PIDController pidController = new PIDController(kArmP, kArmI, kArmD);
    //ToDo Make FF less aggressive
    public final ArmFeedforward controller = new ArmFeedforward(0.095123, 0.51681 - 0.4, 2.4861 - 0.7, 0.13875);
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(armAbsoluteEncoderPort);
    private Constants.ArmPosition armPosition = Constants.ArmPosition.INTAKE;
    private Constants.ExtensionPosition extensionPosition = Constants.ExtensionPosition.RESET;
    // Toggle for Cone or Cube positions
    private boolean cones = false;

    ArmSubsystem() {
        armMotor.configFactoryDefault();
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        armMotor.setSensorPhase(false);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.setInverted(true);

        armMotor.setSelectedSensorPosition(0.0, 0, 10);

        armMotor.configAllowableClosedloopError(0, 10, Constants.kTimeoutMs);

        extension.restoreFactoryDefaults();
        extension.setInverted(true);
    }

    public void setArmMotor(double pos) {
        double armPos = pos + armEncoderOffset;
        armMotor.setVoltage(pidController.calculate(getArmEncoderPosition(), armPos) + controller.calculate(Math.toRadians(armPos), 1.0));
    }

    public void setExtensionMotor(double speed) {
        extension.set(!getLimitSwitch() && speed < 0.0 ? 0.0 : speed);
    }

    public double getExtensionEncoderPosition() {
        return extension.getEncoder().getPosition();
    }

    public void resetExtensionMotor() {
        extension.getEncoder().setPosition(0.0);
    }

    public double getArmEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition() * 360.0;
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Extension Limit Switch", getLimitSwitch());
        SmartDashboard.putNumber("Extension Encoder", getExtensionEncoderPosition());
        SmartDashboard.putNumber("Arm Encoder", getArmEncoderPosition());
        SmartDashboard.putString("Arm Pos", getArmPosition().name());
        SmartDashboard.putString("Extension Pos", getExtensionPosition().name());
    }

    public void setCones(boolean cones) {
        this.cones = cones;
    }

    public boolean getCones() {
        return cones;
    }

    public void setArmPosition(ArmPosition armPosition) {
        this.armPosition = armPosition;
    }

    public ArmPosition getArmPosition() {
        return armPosition;
    }

    public void setExtensionPosition(ExtensionPosition extensionPosition) {
        this.extensionPosition = extensionPosition;
    }

    public ExtensionPosition getExtensionPosition() {
        return extensionPosition;
    }

    public static ArmSubsystem getInstance() {
        if(instance != null)
            return instance;
        return new ArmSubsystem();
    }
}
