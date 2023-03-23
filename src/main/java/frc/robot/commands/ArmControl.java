package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmControl extends CommandBase {
    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final Timer timer = new Timer();
    private final DoubleSupplier armStick, extensionStick;
    
    public ArmControl(DoubleSupplier armStick, DoubleSupplier extensionStick) {
        addRequirements(m_armSubsystem);
        
        this.armStick = armStick;
        this.extensionStick = extensionStick;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        
        if(armStick.getAsDouble() > 0.5 && timer.get() > 0.25) {
            switch (m_armSubsystem.getArmPosition()) {
                case CONE_HIGH:
                case CUBE_HIGH:
                case SLIDE_GRAB:
                    m_armSubsystem.setArmPosition(m_armSubsystem.getCones() ? Constants.ArmPosition.CONE_MEDIUM : Constants.ArmPosition.CUBE_MEDIUM);
                    break;
                case CONE_MEDIUM:
                case CUBE_MEDIUM:
                    m_armSubsystem.setArmPosition(m_armSubsystem.getCones() ? Constants.ArmPosition.CONE_LOW : Constants.ArmPosition.CUBE_LOW);
                    break;
                case CONE_LOW:
                case CUBE_LOW:
                    m_armSubsystem.setArmPosition(Constants.ArmPosition.INTAKE);
                    break;
            }
            timer.reset();
        } else if(armStick.getAsDouble() < -0.5 && timer.get() > 0.25) {
            switch (m_armSubsystem.getArmPosition()) {
                case START:
                    m_armSubsystem.setArmPosition(Constants.ArmPosition.INTAKE);
                    break;
                case INTAKE:
                    m_armSubsystem.setArmPosition(m_armSubsystem.getCones() ? Constants.ArmPosition.CONE_LOW : Constants.ArmPosition.CUBE_LOW);
                    break;
                case CONE_LOW:
                case CUBE_LOW:
                    m_armSubsystem.setArmPosition(m_armSubsystem.getCones() ? Constants.ArmPosition.CONE_MEDIUM : Constants.ArmPosition.CUBE_MEDIUM);
                    break;
                case CONE_MEDIUM:
                case CUBE_MEDIUM:
                case SLIDE_GRAB:
                    m_armSubsystem.setArmPosition(m_armSubsystem.getCones() ? Constants.ArmPosition.CONE_HIGH : Constants.ArmPosition.CUBE_HIGH);
                    break;
            }
            timer.reset();
        }
        
        m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
        
        if(!m_armSubsystem.getExtensionPosition().equals(Constants.ExtensionPosition.RESET))
            m_armSubsystem.setExtensionMotor(Math.abs(extensionStick.getAsDouble()) > 0.3 ?
                    extensionStick.getAsDouble() * -1.0 * 0.8 : 0.0);
        else {
            if(!m_armSubsystem.getLimitSwitch())
                m_armSubsystem.setExtensionMotor(-0.2);
            else {
                m_armSubsystem.resetExtensionMotor();
                m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
            }
        }

        SmartDashboard.putNumber("Timer", timer.get());
    }
}
