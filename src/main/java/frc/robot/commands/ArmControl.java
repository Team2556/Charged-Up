package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class ArmControl extends CommandBase {
    private final ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    private final ClawSubsystem m_clawSubsystem = ClawSubsystem.getInstance();
    private final Timer timer = new Timer();
    private final Timer coneTimer = new Timer();
    private final DoubleSupplier armStick, extensionStick;

    private static ArmState armState = ArmState.MANUAL;
    private static ExtendArmAuto extendarmauto = ExtendArmAuto.RESET;
    private static ConeUpAuto coneAuto = ConeUpAuto.TIMER_RESET;
    private boolean firstAutoLoop = true;
    

    public enum ArmState {
        MANUAL,
        CONE_UP_PICKUP,
        AUTO_PICKUP2
    }
    public enum ExtendArmAuto {
        RESET,
        EXTEND,
        GRAB,
        RETRACT
    }
    public enum ConeUpAuto {
        TIMER_RESET,
        BACK,
        CLOSE,
        FORWARDSLOW,
        BACK2,
        EXTEND,
        GRAB,
        RETRACT
    }

    
    
    public ArmControl(DoubleSupplier armStick, DoubleSupplier extensionStick) {
        addRequirements(m_armSubsystem);
        
        this.armStick = armStick;
        this.extensionStick = extensionStick;
    }

    @Override
    public void initialize() {
        timer.start();
        coneTimer.start();
        firstAutoLoop = true;
                
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

            case CONE_UP_PICKUP:
                    if(Math.abs(extensionStick.getAsDouble()) > 0.5) {
                armState = ArmState.MANUAL;
                //m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
                coneAuto = ConeUpAuto.TIMER_RESET;
                m_armSubsystem.armspeed = 1.0;
                }
                switch(coneAuto) {
                    case TIMER_RESET:
                        if (m_armSubsystem.getLimitSwitch())
                            m_armSubsystem.setExtensionMotor(-0.25);
                        else {
                            m_armSubsystem.resetExtensionMotor();
                            m_armSubsystem.setExtensionMotor(0);
                            m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                            coneTimer.reset();
                            coneAuto = ConeUpAuto.BACK;
                        }
                        break;
                    case BACK:
                        if (coneTimer.get() < .5) {
                            //m_armSubsystem.armspeed = 1;
                            m_armSubsystem.setExtensionMotor(0);
                            m_armSubsystem.setArmMotor(-122);
                        } else {
                            coneAuto = ConeUpAuto.CLOSE;
                            coneTimer.reset();
                        }
                    break;
                    case CLOSE: 
                            m_clawSubsystem.clawCloseAction();
                            m_armSubsystem.armspeed = 0.3;
                            coneAuto = ConeUpAuto.FORWARDSLOW;
                            coneTimer.reset();
                    break;
                    case FORWARDSLOW:
                        if (coneTimer.get() < 0.5) {
                            m_armSubsystem.setArmMotor(-92);
                        } else {
                            coneAuto = ConeUpAuto.BACK2;
                            coneTimer.reset();
                        }
                    break;
                    case BACK2:
                        if (coneTimer.get() < 0.5) {
                            m_armSubsystem.setArmMotor(-97);
                        } else {
                            coneAuto = ConeUpAuto.EXTEND;
                            coneTimer.reset();
                        } 
                    break;
                    case EXTEND:
                            if (m_armSubsystem.getExtensionEncoderPosition()  <= 70) {
                                m_armSubsystem.setExtensionMotor(0.45);
                                m_clawSubsystem.clawOpenAction();
                            } else {
                                m_armSubsystem.setExtensionMotor(0);
                                coneAuto = ConeUpAuto.GRAB;
                            coneTimer.reset();
                            }
                    break;
                    case GRAB:
                        
                            m_clawSubsystem.clawCloseAction();
                            m_clawSubsystem.runClawForSeconds(2, 0.75);
                       
                            coneAuto = ConeUpAuto.RETRACT;
                            coneTimer.reset();
                         
                    break;
                    case RETRACT:
                            if (m_armSubsystem.getLimitSwitch()) {
                                m_armSubsystem.setExtensionMotor(-0.45);
                                m_clawSubsystem.runClawForSeconds(0.6, 0.4);
                            }
                            else {
                                m_armSubsystem.resetExtensionMotor();
                                m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);m_armSubsystem.setArmMotor(m_armSubsystem.getArmPosition().getPosition());
                                m_armSubsystem.armspeed = 1.0;
                                coneAuto = ConeUpAuto.TIMER_RESET;
                                armState = ArmState.MANUAL;
                                coneTimer.reset();
                        }
                    break;
                }
            break;

            case AUTO_PICKUP2:
            if(Math.abs(extensionStick.getAsDouble()) > 0.5) {
                armState = ArmState.MANUAL;
                extendarmauto = ExtendArmAuto.RESET;
            }
             switch(extendarmauto){
                case RESET:
                    m_clawSubsystem.clawOpenAction();
                    m_armSubsystem.setExtensionPosition(m_armSubsystem.getCones() ? Constants.ExtensionPosition.GRAB_CONE : Constants.ExtensionPosition.GRAB_CUBE);
                    if (m_armSubsystem.getLimitSwitch()||!m_armSubsystem.getArmPosition().equals(Constants.ArmPosition.GRAB))
                        armState = ArmState.MANUAL;
                    else{
                    extendarmauto = ExtendArmAuto.EXTEND;
                    }
                break;
                case EXTEND:
                    if (m_armSubsystem.getExtensionEncoderPosition() <= m_armSubsystem.getExtensionPosition().getPosition()){
                    m_armSubsystem.setExtensionMotor(0.45);
                    }

                    else {
                    m_armSubsystem.setExtensionMotor(0.0);
                    extendarmauto = ExtendArmAuto.GRAB;
                    }

                    if(!m_armSubsystem.getCones())
                    m_clawSubsystem.clawPullAction();
                break;
                case GRAB:
                    m_clawSubsystem.resetTimer();
                    if(m_armSubsystem.getCones())
                            m_clawSubsystem.clawCloseAction();
                    extendarmauto = ExtendArmAuto.RETRACT;
                break;
                case RETRACT:
                    if (m_armSubsystem.getLimitSwitch()) {
                     m_armSubsystem.setExtensionMotor(-0.45);
                     m_clawSubsystem.runClawForSeconds(0.6, 0.4);
                    }
                     else {
                     m_armSubsystem.resetExtensionMotor();
                     m_armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                     extendarmauto = ExtendArmAuto.RESET;
                     armState = ArmState.MANUAL;
                     }
                break;
             }
             }

    if(Math.abs(armStick.getAsDouble()) > 0.5 || Math.abs(extensionStick.getAsDouble()) > 0.5 || !m_armSubsystem.getArmPosition().equals(Constants.ArmPosition.GRAB)) {
        armState = ArmState.MANUAL;
        return;
    }

    
    }
    public static void setArmState(ArmState armState) {
        ArmControl.armState = armState;
    }



    private boolean almostEqual(double a, double b, double eps){
        return Math.abs(a-b) < eps;
    }
}

