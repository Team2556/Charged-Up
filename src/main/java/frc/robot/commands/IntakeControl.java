package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeControl extends CommandBase {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final DoubleSupplier rightTrigger, leftTrigger;

    public IntakeControl(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        addRequirements(intakeSubsystem);

        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeOff();
    }

    @Override
    public void execute() {
        if(rightTrigger.getAsDouble() > 0.5) {
            intakeSubsystem.setIntakeMotor(rightTrigger.getAsDouble() * 0.4);
            intakeSubsystem.intakeExtend();
        } else if(leftTrigger.getAsDouble() > 0.5) {
            intakeSubsystem.setIntakeMotor(leftTrigger.getAsDouble() * -0.4);
            intakeSubsystem.intakeExtend();
        } else {
            intakeSubsystem.setIntakeMotor(0.0);
            intakeSubsystem.intakeRetract();
        }
    }
}
