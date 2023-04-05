package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveForward extends CommandBase {
    private final Timer timer = new Timer();
    private boolean firstRun = true, done = false;
    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final SwerveSubsystem m_swerveSubsystem = SwerveSubsystem.getInstance();

    private final double delay = 5.0;
    private final double speed = -0.4;

    @Override
    public void initialize() {
        timer.start();
        firstRun = true;
        done = false;
    }

    @Override
    public void execute() {
       if(firstRun) {
            timer.reset();
            m_swerveSubsystem.setIsFieldRelative(false);
            firstRun = false;
        }

        if (m_armSubsystem.getLimitSwitch())
            m_armSubsystem.setExtensionMotor(-0.25);
        else {
            m_armSubsystem.resetExtensionMotor(); 
            m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
        }
        SmartDashboard.putNumber("Auto Timer", timer.get());
        if(timer.get() < delay)
            m_swerveSubsystem.drive(speed, 0.0, 0.0, true);
        else {
            m_swerveSubsystem.drive(0.0, 0.0, 0.0, true);
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(0.0, 0.0, 0.0, true);
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
