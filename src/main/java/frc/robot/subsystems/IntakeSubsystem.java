package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.intakeMotorPort;

/**
 * Intake Subsystem
 *  - Intake Neo Motor
 *  - Solenoids to extend/retract the intake
 */
//ToDo Add Solenoids
public class IntakeSubsystem extends SubsystemBase {
    private final static IntakeSubsystem instance = getInstance();
    private final CANSparkMax intake = new CANSparkMax(intakeMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    public IntakeSubsystem() {
        intake.restoreFactoryDefaults();
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setIntakeMotor(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        intake.set(speed);
    }

    public static IntakeSubsystem getInstance() {
        if(instance == null)
            return new IntakeSubsystem();
        return instance;
    }
}
