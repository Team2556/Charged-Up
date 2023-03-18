package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Commands {

    private Object gyro;
public boolean pressed;
public class Intake extends SubsystemBase {
    // [code for motor controllers, configuration, etc.]
    // ...

    public Object stopCommand;

    public Command runIntakeCommand() {
      // implicitly requires `this`
      return this.startEnd(() -> this.set(1.0), () -> this.set(0.0));
    }

    public Object set(double d) {
        return null;
    }

    public Object runIntakeCommand(double d) {
        return null;
    }

    public void stopCommand() {
    }
}

public class AutoRoutines {

    public Command Intake(Intake intake) {
        return Commands.sequence(
            ((Command) Commands.parallel(
                intake.runIntakeCommand(1.0)
            )).withTimeout(5.0),
            Commands.parallel(gyro));
 
        
    }
}

    public static Command startEnd(Object set, Intake set2, Object intake) {
        return null;
    }

    public static Command startEnd(Object set, Object set2, Object intake) {
        return null;
    }

    public static Object waitSeconds(double d) {
        return null;
    }

    public static Command sequence(ParallelRaceGroup withTimeout, Object waitSeconds, ParallelRaceGroup withTimeout2) {
        return null;
    }

    public static Command parallel(Object runIntakeCommand) {
        return null;
    }

    public static Command sequence(ParallelRaceGroup withTimeout, Command parallel) {
        return null;
    }
    private static final int kMotorPort = 0;
    private static final int kEncoderPortA = 0;
    private static final int kEncoderPortB = 1;
    private static final Intake intake = null;
    Command runIntake = Commands.startEnd(((Intake) intake).set(1), (Intake)((Intake) intake).set(0), intake);
    Command autonomousCommand = Commands.sequence(
    Commands.startEnd(((Intake) intake).set(1.0), ((Intake) intake).set(0.0), intake).withTimeout(5.0),
    Commands.waitSeconds(3.0),
    Commands.startEnd( ((Intake) intake).set(1.0), ( ((Intake) intake).set(0.0)), intake).withTimeout(5.0));
    public Object xButton;


    public class RunIntakeCommand extends CommandBase {
        private Intake m_intake;
        private Command runIntake;
    
        public RunIntakeCommand(Intake intake) {
            this.m_intake = intake;
            addRequirements(intake);
        }
    
        private void addRequirements(Intake intake) {
        }

        @Override
        public void initialize() {
            m_intake.set(1.0);
        }
    
        
        {((RunIntakeCommand) xButton).whenPressed(runIntake);
        
        
    
    
          }
    
        @Override
        public void end(boolean interrupted) {
            m_intake.set(0.0);
        }
        private void whenPressed(Command runIntake2) {
        }
        Command stopIntake;
       {((RunIntakeCommand) xButton).whenReleased(stopIntake);}

    private void whenReleased(Command stopIntake2) {
    }

    }

    public void set(double d) {
    }
    
    private static Command startEnd(Object set, Intake set2, Intake intake2) {
        return null;
    }
}

 

