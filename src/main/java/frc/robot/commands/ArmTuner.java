package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

public class ArmTuner extends CommandBase {
    private static final Arm arm = Arm.getInstance();
    public static double pos = 0.0;
    private final DoubleSupplier extensionStick;

    public ArmTuner(DoubleSupplier extensionStick) {
        addRequirements(arm);
        SmartDashboard.putNumber("Arm P", 0.0);
        SmartDashboard.putNumber("Arm I", 0.0);
        SmartDashboard.putNumber("Arm D", 0.0);
        SmartDashboard.putNumber("Arm F", 0.0);
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
            arm.pidController.setP(p);
            arm.pidController.setI(i);
            arm.pidController.setD(d);
        }

        if(extensionStick.getAsDouble() > 0.5)
            arm.setExtensionMotor(extensionStick.getAsDouble() * 0.8);
        else if(extensionStick.getAsDouble() < -0.5 && arm.getLimitSwitch())
            arm.setExtensionMotor(extensionStick.getAsDouble() * 0.8);
        else
            arm.setExtensionMotor(0.0);

        arm.setArmMotor(pos);
        SmartDashboard.putBoolean("Extension Limit Switch", arm.getLimitSwitch());
        SmartDashboard.putNumber("Extension Encoder", arm.getExtensionMotorEncoder());
        SmartDashboard.putNumber("Arm Encoder", arm.getEncoderPosition());
    }
}
