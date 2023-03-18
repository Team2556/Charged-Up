package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCone extends CommandBase {
    Intake intake;
    double timeout = -1;
    Timer timer = new Timer();

    public IntakeCone(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    private void addRequirements(frc.robot.commands.Intake intake2) {
    }

    // timeout in seconds
    public IntakeCone(Intake intake, double timeout) {
        this.intake = intake;
        this.timeout = timeout;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        ((Intake) intake).runIntakeCone();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return timeout == -1 ^ timer.get() > timeout;
    }

}