package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeCone extends CommandBase {
    Intake intake;
    double timeout = -1;
    Timer timer = new Timer();

    public void RunOuttakeCone(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    // Timeout in seconds
    private void addRequirements(Intake intake2) {
    }

    public void RunOuttakeCone(Intake intake, double timeout) {
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
        intake.runOuttakeCone();
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