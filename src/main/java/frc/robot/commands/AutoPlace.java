package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPlace extends CommandBase {
    
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ClawSubsystem clawSubsystem = ClawSubsystem.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private boolean finished = false, firstLoop = true, clawRun = false;
    private State state = State.START;
    private final Timer timer = new Timer();

    private enum State {
        START,
        GRAB,
        ARM_PLACE_PREP,
        PLACE,
        PARK
    }

    @Override
    public void initialize() {
        finished = false;
        firstLoop = true;
        timer.start();
        armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RESET);
    }

    @Override
    public void execute() {
        switch (state) {
            case START:
            
                if(firstLoop) {
                    swerveSubsystem.resetTimer();
                    swerveSubsystem.setIsFieldRelative(false);
                    armSubsystem.setArmPosition(Constants.ArmPosition.START);
                    armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RESET);
                    firstLoop = false;
                }

                if(armSubsystem.getExtensionPosition().equals(Constants.ExtensionPosition.RESET)) {
                    if (armSubsystem.getLimitSwitch())
                        armSubsystem.setExtensionMotor(-0.25);
                    else {
                        armSubsystem.resetExtensionMotor();
                        armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                        state = State.GRAB;
                        firstLoop = true;
                        return;
                    }
                }

                armSubsystem.setArmMotor(armSubsystem.getArmPosition().getPosition());
                break;
            case GRAB:
                if(firstLoop) {
                    clawSubsystem.clawOpenAction();
                    timer.reset();
                    armSubsystem.setExtensionPosition(armSubsystem.getCones() ? Constants.ExtensionPosition.GRAB_CONE : Constants.ExtensionPosition.GRAB_CUBE);
                    firstLoop = false;
                    clawRun = false;
                }

                if(almostEqual(armSubsystem.getExtensionPosition().getPosition(), armSubsystem.getExtensionEncoderPosition(), 2)
                && armSubsystem.getExtensionPosition().equals(armSubsystem.getCones() ? Constants.ExtensionPosition.GRAB_CONE : Constants.ExtensionPosition.GRAB_CUBE)) {
                    clawSubsystem.resetTimer();
                    if(armSubsystem.getCones())
                        clawSubsystem.clawCloseAction();
                    armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                }

                if(armSubsystem.getCones() &&
                        armSubsystem.getExtensionPosition().equals(Constants.ExtensionPosition.RETRACT) &&
                        !almostEqual(armSubsystem.getExtensionPosition().getPosition(), armSubsystem.getExtensionEncoderPosition(), 8)) {
                    clawSubsystem.runClawForSeconds(0.6, 0.4);
                    clawRun = true;
                    timer.reset();
                }

                if(!armSubsystem.getCones())
                    clawSubsystem.clawPullAction();
                armSubsystem.setExtensionPositionPID(armSubsystem.getExtensionPosition().getPosition());
                if(clawRun && armSubsystem.getExtensionPosition().equals(Constants.ExtensionPosition.RETRACT) && timer.get() > 0.6) {
                    state = State.ARM_PLACE_PREP;
                    firstLoop = true;
                    return;
                }
                break;
            case ARM_PLACE_PREP:
                if(firstLoop) {
                    armSubsystem.setArmPosition(Constants.ArmPosition.CONE_HIGH);
                    armSubsystem.setExtensionPosition(Constants.ExtensionPosition.PLACE_CONE);
                    firstLoop = false;
                }

                if(almostEqual(armSubsystem.getArmEncoderPosition(), armSubsystem.getArmPosition().getPosition(), 3.0)
                    && almostEqual(armSubsystem.getExtensionEncoderPosition(), armSubsystem.getExtensionPosition().getPosition(), 5.0)) {
                    state = State.PLACE;
                    firstLoop = true;
                    return;
                }

                armSubsystem.setArmMotor(armSubsystem.getArmPosition().getPosition());
                armSubsystem.setExtensionPositionPID(armSubsystem.getExtensionPosition().getPosition());
                break;
            case PLACE:
                if(firstLoop) {
                    armSubsystem.setArmPosition(Constants.ArmPosition.CONE_HIGH);
                    armSubsystem.setExtensionPosition(Constants.ExtensionPosition.PLACE_CONE);
                    timer.reset();
                    firstLoop = false;
                }

                clawSubsystem.clawOpenAction();

                if(timer.get() > 1.0) {
                    state = State.PARK;
                    firstLoop = true;
                }
                break;
            case PARK:
                if(firstLoop) {
                    timer.reset();
                    firstLoop = false;
                    armSubsystem.setExtensionPosition(Constants.ExtensionPosition.RETRACT);
                }

                if(timer.get() > 1.5 && armSubsystem.getArmPosition().equals(Constants.ArmPosition.CONE_HIGH)) {
                    armSubsystem.setArmPosition(Constants.ArmPosition.INTAKE);
                }

                armSubsystem.setArmMotor(armSubsystem.getArmPosition().getPosition());
                armSubsystem.setExtensionPositionPID(armSubsystem.getExtensionPosition().getPosition());
                break;
        }

        SmartDashboard.putString("Auto State", state.name());
    }

    @Override
    public boolean isFinished() {
        return finished;
    }


    private boolean almostEqual(double a, double b, double eps){
        return Math.abs(a-b) < eps;
    }
}
