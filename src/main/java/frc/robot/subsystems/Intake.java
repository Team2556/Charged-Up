package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final static Intake instance = getInstance();
    private final CANSparkMax intake = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Intake() {
        intake.restoreFactoryDefaults();
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setIntakeMotor(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        intake.set(speed);
    }

    public static Intake getInstance() {
        if(instance == null)
            return new Intake();
        return instance;
    }
}
