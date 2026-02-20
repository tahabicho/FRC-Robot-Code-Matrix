package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class Robot extends TimedRobot {

  // ========= CONSTANTES =========
  private static final MotorType MOTOR_TYPE  = MotorType.kBrushed;
  private static final int CURRENT_LIMIT     = (MOTOR_TYPE == MotorType.kBrushless) ? 40 : 35;

  private static final double NORMAL_SPEED   = 0.6;
  private static final double SLOW_SPEED     = 0.3;
  private static final double TURBO_SPEED    = 1.0;
  private static final double INTAKE_SPEED   = 0.7;
  private static final double OUTAKE_SPEED   = 0.7;

  private static final double ROTATION_SCALE = 0.75;
  private static final double DEADBAND       = 0.08;

  // ========= MODES DE VITESSE =========
  private enum SpeedMode { SLOW, NORMAL, TURBO }
  private SpeedMode currentSpeedMode = SpeedMode.NORMAL;

  // ========= ÉTAT BOUTONS (edge detection) =========
  private boolean prevBtnA = false;
  private boolean prevBtnX = false;

  // ========= MATÉRIEL =========
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkMax outakeMotor;  // ID 2 — Bouton A
  private SparkMax intakeMotor;  // ID 3 — Bouton X
  private DifferentialDrive drive;
  private XboxController controller;

  // Limiteurs d'accélération
  private final SlewRateLimiter speedLimiter    = new SlewRateLimiter(2.5);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.5);

  // =========================================================
  // INIT ROBOT
  // =========================================================
  @Override
  public void robotInit() {
    leftMotor   = createConfiguredMotor(1, false);
    rightMotor  = createConfiguredMotor(4, true);
    outakeMotor = createConfiguredMotor(2, false);
    intakeMotor = createConfiguredMotor(3, false);

    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setDeadband(DEADBAND);
    drive.setSafetyEnabled(true);

    controller = new XboxController(0);

    // Valeurs initiales SmartDashboard
    SmartDashboard.putString ("Speed Mode",       "NORMAL");
    SmartDashboard.putBoolean("Outake (A)",        false);
    SmartDashboard.putBoolean("Intake (X)",        false);
    SmartDashboard.putBoolean("Speed Cycle (B)",   false);
    SmartDashboard.putString ("Robot Status",      "Ready");

    System.out.println("========================================");
    System.out.println("[ROBOT] Initialisation terminée");
    System.out.println("[ROBOT] Mode vitesse initial : NORMAL");
    System.out.println("========================================");
  }

  // =========================================================
  // INIT TELEOP — reset limiteurs pour éviter les sauts
  // =========================================================
  @Override
  public void teleopInit() {
    speedLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
    drive.stopMotor();
    outakeMotor.set(0.0);
    intakeMotor.set(0.0);
    System.out.println("[TELEOP] Démarrage téléopéré — limiteurs réinitialisés");
  }

  // =========================================================
  // INIT AUTONOME — sécuriser intake/outake
  // =========================================================
  @Override
  public void autonomousInit() {
    outakeMotor.set(0.0);
    intakeMotor.set(0.0);
    System.out.println("[AUTO] Démarrage autonome");
  }

  // =========================================================
  // TELEOP PERIODIC
  // =========================================================
  @Override
  public void teleopPeriodic() {

    // --- Lecture boutons ---
    boolean btnA = controller.getAButton();  // Outake (moteur ID 2)
    boolean btnX = controller.getXButton();  // Intake  (moteur ID 3)

    // --- Cycle speed mode : front montant de B ---
    if (controller.getBButtonPressed()) {
      switch (currentSpeedMode) {
        case SLOW:   currentSpeedMode = SpeedMode.NORMAL; break;
        case NORMAL: currentSpeedMode = SpeedMode.TURBO;  break;
        case TURBO:  currentSpeedMode = SpeedMode.SLOW;   break;
      }
      System.out.println("[MODE] Vitesse changée → " + currentSpeedMode.name());
    }

    // --- Outake (A) : log uniquement sur changement d'état ---
    outakeMotor.set(btnA ? OUTAKE_SPEED : 0.0);
    if (btnA != prevBtnA) {
      System.out.println(btnA ? "[OUTAKE] Moteur ID 2 → ON" : "[OUTAKE] Moteur ID 2 → OFF");
      prevBtnA = btnA;
    }

    // --- Intake (X) : log uniquement sur changement d'état ---
    intakeMotor.set(btnX ? INTAKE_SPEED : 0.0);
    if (btnX != prevBtnX) {
      System.out.println(btnX ? "[INTAKE] Moteur ID 3 → ON" : "[INTAKE] Moteur ID 3 → OFF");
      prevBtnX = btnX;
    }

    // --- Conduite différentielle ---
    double speed    = -applyDeadband(controller.getRawAxis(1));
    double rotation =  applyDeadband(controller.getRawAxis(4)) * ROTATION_SCALE;
    speed    = speedLimiter.calculate(speed);
    rotation = rotationLimiter.calculate(rotation);

    drive.setMaxOutput(getSpeedValue());
    drive.arcadeDrive(speed, rotation);

    // ─── SmartDashboard ───────────────────────────────────────
    SmartDashboard.putString ("Speed Mode",          currentSpeedMode.name());
    SmartDashboard.putBoolean("Outake (A)",           btnA);
    SmartDashboard.putBoolean("Intake (X)",           btnX);
    SmartDashboard.putBoolean("Speed Cycle (B)",      controller.getBButton());
    SmartDashboard.putNumber ("Speed",                speed);
    SmartDashboard.putNumber ("Rotation",             rotation);
    SmartDashboard.putNumber ("Left Current (A)",     leftMotor.getOutputCurrent());
    SmartDashboard.putNumber ("Right Current (A)",    rightMotor.getOutputCurrent());
    SmartDashboard.putNumber ("Outake Current (A)",   outakeMotor.getOutputCurrent());
    SmartDashboard.putNumber ("Intake Current (A)",   intakeMotor.getOutputCurrent());
  }

  // =========================================================
  // AUTONOME PERIODIC
  // =========================================================
  @Override
  public void autonomousPeriodic() {
    drive.arcadeDrive(0.35, 0.0);
  }

  // =========================================================
  // DISABLED
  // =========================================================
  @Override
  public void disabledInit() {
    drive.stopMotor();
    outakeMotor.set(0.0);
    intakeMotor.set(0.0);
    System.out.println("[ROBOT] Désactivé — tous les moteurs arrêtés");
  }

  // =========================================================
  // OUTILS
  // =========================================================
  private SparkMax createConfiguredMotor(int id, boolean inverted) {
    SparkMax motor = new SparkMax(id, MOTOR_TYPE);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(inverted);
    config.voltageCompensation(12.0);
    config.openLoopRampRate(0.25);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private double applyDeadband(double value) {
    return Math.abs(value) < DEADBAND ? 0 : value;
  }

  private double getSpeedValue() {
    switch (currentSpeedMode) {
      case SLOW:  return SLOW_SPEED;
      case TURBO: return TURBO_SPEED;
      default:    return NORMAL_SPEED;
    }
  }
}
