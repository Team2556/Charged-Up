package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.Commands;

public class Itake {
    private static final String CANSparkMaxLowLevel = null;
    
    private Object intake;
    
    private Encoder m_encoder;
    XboxController exampleXbox = new XboxController(0);
    private Object exampleCommandController;
    Trigger xButton = ((Trigger) exampleCommandController).RT(); 
    private int m_notifier;
    private int kMotorPort;
    private DigitalSource kEncoderPortA;
    private DigitalSource kEncoderPortB;
    private static CANSparkMax itake = new CANSparkMax(CANSparkMaxLowLevel);


    public void robotInit() {
      new PWMSparkMax(kMotorPort);
      new JoystickButton(exampleXbox, m_notifier);
      m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
      // Use SetDistancePerPulse to set the multiplier for GetDistance
      // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
      m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);

      
    ;}

public int turnToAngle(double targetDegrees) {
    try (// Create a controller for the inline command to capture
    PIDController controller = new PIDController(Constants.kTurnToAngleP, 0, 0)) {
        // We can do whatever configuration we want on the created state before returning from the factory
        controller.setTolerance((double) Constants.kTurnToAngleTolerance);
    }
    return 0;

    // Try to turn at a rate proportional to the heading error until we're at the setpoint, then stop
   


}
}



