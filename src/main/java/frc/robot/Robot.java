package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private PWMSparkMax m_leftMotor;
  private PWMSparkMax m_rightMotor;

  @Override
  public void robotInit() {
    m_leftMotor = new PWMSparkMax(1);
    m_rightMotor = new PWMSparkMax(4);
    
    m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_myRobot.setDeadband(0.05);
    
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
  }
}
