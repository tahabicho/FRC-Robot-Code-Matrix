package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_stick;
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  
  // ===============================================
  // CONFIGURATION - Changez selon vos moteurs physiques
  // ===============================================
  private static final MotorType MOTOR_TYPE = MotorType.kBrushed;
  
  // Paramètres selon le type de moteur
  private static final int CURRENT_LIMIT = (MOTOR_TYPE == MotorType.kBrushless) ? 40 : 35;
  
  private static final double NORMAL_SPEED = 0.2;
  private static final double SLOW_SPEED = 0.2;
  private static final double TURBO_SPEED = 0.2;
  private static final double ROTATION_SPEED = 0.7;
  private static final double DEADBAND = 0.08;

  @Override
  public void robotInit() {
    // Initialisation des moteurs avec le type configuré
    m_leftMotor = new SparkMax(1, MOTOR_TYPE);
    m_rightMotor = new SparkMax(4, MOTOR_TYPE);
    
    // Configuration moteur gauche
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.voltageCompensation(12.0);
    config.openLoopRampRate(0.3);
    
    m_leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    // Configuration moteur droit (inversé)
    SparkMaxConfig configRight = new SparkMaxConfig();
    configRight.smartCurrentLimit(CURRENT_LIMIT);
    configRight.idleMode(IdleMode.kBrake);
    configRight.inverted(true);
    configRight.voltageCompensation(12.0);
    configRight.openLoopRampRate(0.3);
    
    m_rightMotor.configure(configRight, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Configuration du DifferentialDrive
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_myRobot.setDeadband(DEADBAND);
    m_myRobot.setMaxOutput(NORMAL_SPEED);
    
    m_stick = new Joystick(0);
    
    // Affichage du type de moteur sur SmartDashboard
    String motorTypeName = (MOTOR_TYPE == MotorType.kBrushless) ? "Brushless" : "Brushed";
    SmartDashboard.putString("Motor Type", motorTypeName);
    SmartDashboard.putNumber("Current Limit", CURRENT_LIMIT);
    SmartDashboard.putString("Control Mode", "Split Arcade (LY + RX)");
  }

  @Override
  public void teleopPeriodic() {
    // CHANGEMENT ICI: LY pour avant/arrière, RX pour rotation
    double speed = -m_stick.getRawAxis(1);      // Axe 1 = Stick Gauche Y (LY)
    double rotation = -m_stick.getRawAxis(4);   // Axe 4 = Stick Droit X (RX)
    
    // Modes de vitesse
    if (m_stick.getRawButton(1)) {
      m_myRobot.setMaxOutput(TURBO_SPEED);
      SmartDashboard.putString("Speed Mode", "TURBO");
    } else if (m_stick.getRawButton(2)) {
      m_myRobot.setMaxOutput(SLOW_SPEED);
      SmartDashboard.putString("Speed Mode", "SLOW");
    } else {
      m_myRobot.setMaxOutput(NORMAL_SPEED);
      SmartDashboard.putString("Speed Mode", "NORMAL");
    }
    
    m_myRobot.arcadeDrive(speed, rotation);
    
    // Télémétrie
    SmartDashboard.putNumber("LY (Speed)", speed);
    SmartDashboard.putNumber("RX (Rotation)", rotation);
    SmartDashboard.putNumber("Left Motor Current", m_leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Motor Current", m_rightMotor.getOutputCurrent());
  }

  @Override
  public void disabledInit() {
    m_myRobot.stopMotor();
  }
}
