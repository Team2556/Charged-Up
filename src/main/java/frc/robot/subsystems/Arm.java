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

public class Arm extends SubsystemBase {
    private static final Arm instance = getInstance();
    private final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.armMotorPort);
    private final CANSparkMax extension = new CANSparkMax(Constants.extensionMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(4);
    public final PIDController pidController = new PIDController(0.0, 0.0, 0.0);
    public final ArmFeedforward controller = new ArmFeedforward(0.095123, 0.51681 - 0.4, 2.4861 - 0.7, 0.13875);
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);

    Arm() {
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
        double armPos = pos + Constants.armEncoderOffset;
        SmartDashboard.putNumber("FF Position", armPos);

        armMotor.setVoltage(pidController.calculate(getEncoderPosition(), armPos) + controller.calculate(Math.toRadians(armPos), 1.0));
    }

    public void setExtensionMotor(double speed) {
        extension.set(speed);
    }

    public double getExtensionMotorEncoder() {
        return extension.getEncoder().getPosition();
    }

    public double getEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition() * 360.0;
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", getEncoderPosition());
    }

    public static Arm getInstance() {
        if(instance != null)
            return instance;
        return new Arm();
    }
}
