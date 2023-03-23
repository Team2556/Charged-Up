package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmTuner extends CommandBase {
    private static final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    public static double pos = -108.0;
    private final DoubleSupplier extensionStick;

    public ArmTuner(DoubleSupplier extensionStick) {
        addRequirements(m_armSubsystem);
        SmartDashboard.putNumber("Arm P", 0.0);
        SmartDashboard.putNumber("Arm I", 0.0);
        SmartDashboard.putNumber("Arm D", 0.0);
        SmartDashboard.putNumber("Arm Position", pos);

        this.extensionStick = extensionStick;
    }

    private double p, i, d, kP, kI, kD;

    @Override
    public void execute() {
        kP = SmartDashboard.getNumber("Arm P", 0.0);
        kI = SmartDashboard.getNumber("Arm I", 0.0);
        kD = SmartDashboard.getNumber("Arm D", 0.0);

        pos = SmartDashboard.getNumber("Arm Position", pos);

        if(p != kP || i != kI || d != kD) {
            p = kP;
            i = kI;
            d = kD;
            m_armSubsystem.armPIDController.setP(p);
            m_armSubsystem.armPIDController.setI(i);
            m_armSubsystem.armPIDController.setD(d);
        }

        m_armSubsystem.setExtensionMotor(Math.abs(extensionStick.getAsDouble()) > 0.3 ?
                extensionStick.getAsDouble() * -1.0 * 0.8 : 0.0);

        m_armSubsystem.setArmMotor(pos);
    }
}
