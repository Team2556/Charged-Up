package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

import java.util.function.DoubleSupplier;

public class ArmControl extends CommandBase {
    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
    private final Timer timer = new Timer();
    private final DoubleSupplier armStick, extensionStick;

    private static ArmState armState = ArmState.MANUAL;
    private boolean firstAutoLoop = true;

    public enum ArmState {
        MANUAL,
        AUTO_PICKUP
    }
    
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
        switch (armState) {
            case MANUAL:
                if (!m_armSubsystem.getExtensionPosition().equals(Constants.ExtensionPosition.RESET))
                    m_armSubsystem.setExtensionMotor(Math.abs(extensionStick.getAsDouble()) > 0.3 ?
                            extensionStick.getAsDouble() * -1.0 * 0.8 : 0.0);
                else {
                    if (m_armSubsystem.getLimitSwitch())
                        m_armSubsystem.setExtensionMotor(-0.25);
                    else {
                        m_armSubsystem.resetExtensionMotor();
                        m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                    }
                }

                if (armStick.getAsDouble() > 0.5 && timer.get() > 0.25) {
                    if (!m_armSubsystem.getLimitSwitch()) {
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
                            case INTAKE:
                                m_armSubsystem.setArmPosition(Constants.ArmPosition.GRAB);
                                break;
                        }
                        timer.reset();
                    }
                } else if (armStick.getAsDouble() < -0.5 && timer.get() > 0.25) {
                    if (!m_armSubsystem.getLimitSwitch()) {
                        switch (m_armSubsystem.getArmPosition()) {
                            case START:
                                m_armSubsystem.setArmPosition(Constants.ArmPosition.GRAB);
                                break;
                            case GRAB:
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
                }

                m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
                firstAutoLoop = true;
                break;
            case AUTO_PICKUP:
                if(Math.abs(armStick.getAsDouble()) > 0.5 || Math.abs(extensionStick.getAsDouble()) > 0.5 || !m_armSubsystem.getArmPosition().equals(Constants.ArmPosition.GRAB)) {
                    armState = ArmState.MANUAL;
                    return;
                }

                if(firstAutoLoop) {
                    m_clawSubsystem.clawOpenAction();
                    m_armSubsystem.setExtensionPosition(m_armSubsystem.getCones() ? Constants.ExtensionPosition.GRAB_CONE : Constants.ExtensionPosition.GRAB_CUBE);
                    firstAutoLoop = false;
                }

                if(almostEqual(m_armSubsystem.getExtensionPosition().getPosition(), m_armSubsystem.getExtensionEncoderPosition(), 3)) {
                    if(m_armSubsystem.getCones())
                        m_clawSubsystem.clawCloseAction();
                    m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                }

                if(!m_armSubsystem.getCones())
                    m_clawSubsystem.clawPullAction();
                m_armSubsystem.setExtensionPositionPID(m_armSubsystem.getExtensionPosition().getPosition());
        }
    }

    public static void setArmState(ArmState armState) {
        ArmControl.armState = armState;
    }

    private boolean almostEqual(double a, double b, double eps){
        return Math.abs(a-b) < eps;
    }
}
