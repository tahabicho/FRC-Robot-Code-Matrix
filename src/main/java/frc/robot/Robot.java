package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class Robot extends TimedRobot {

  // ===================== CONSTANTES =====================

  // CAN IDs — Drive
  private static final int LEFT_LEADER_ID    = 2;
  private static final int LEFT_FOLLOWER_ID  = 6;
  private static final int RIGHT_LEADER_ID   = 1;
  private static final int RIGHT_FOLLOWER_ID = 4;

  // CAN IDs — Intake / Outake
  private static final int INTAKE_ID = 3;
  private static final int OUTAKE_ID = 5;

  // Moteurs
  private static final MotorType MOTOR_TYPE    = MotorType.kBrushed;
  private static final int       CURRENT_LIMIT = 35;

  // Sécurité courant
  private static final double MAX_INTAKE_CURRENT = 30.0;
  private static final double MAX_OUTAKE_CURRENT = 30.0;

  // Brownout
  private static final double BROWNOUT_VOLTAGE = 7.0;

  // Vitesses drive
  private static final double SLOW_SPEED   = 0.50;
  private static final double NORMAL_SPEED = 0.75;
  private static final double TURBO_SPEED  = 1.00;

  // ✅ Intake / Outake toujours à vitesse MAXIMUM (1.0)
  private static final double INTAKE_MAX_SPEED = 1.00;
  private static final double OUTAKE_MAX_SPEED = 1.00;

  // Seuil axe Z / Z Rotation
  private static final double AXIS_THRESHOLD = 0.10;

  // Conduite
  private static final double DEADBAND      = 0.08;
  private static final double SLEW_SPEED    = 3.5;
  private static final double SLEW_ROTATION = 4.5;
  private static final double SLEW_INTAKE   = 5.0;

  // Preferences
  private static final String PREF_SPEED_MODE = "speedMode";

  // ===================== ENUM VITESSE =====================

  private enum SpeedMode { SLOW, NORMAL, TURBO }
  private SpeedMode currentSpeedMode = SpeedMode.TURBO;

  // ===================== MATÉRIEL =====================

  private SparkMax leftLeader,  leftFollower;
  private SparkMax rightLeader, rightFollower;
  private SparkMax intakeMotor, outakeMotor;

  private DifferentialDrive drive;
  private XboxController    controller;
  private Joystick          rawStick;
  private ADXRS450_Gyro     navX;
  private final Timer       autoTimer = new Timer();

  // Limiteurs
  private final SlewRateLimiter speedLimiter    = new SlewRateLimiter(SLEW_SPEED);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SLEW_ROTATION);
  private final SlewRateLimiter intakeLimiter   = new SlewRateLimiter(SLEW_INTAKE);
  private final SlewRateLimiter outakeLimiter   = new SlewRateLimiter(SLEW_INTAKE);

  // État boutons
  private boolean prevA        = false;
  private boolean prevIntakeOn = false;
  private boolean prevOutakeOn = false;

  // Surchauffe
  private boolean intakeOverheat = false;
  private boolean outakeOverheat = false;

  // Gyro
  private boolean tiltAlerted = false;

  // ===================== ROBOT INIT =====================

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("[ROBOT] DataLog démarré");

    RobotController.setBrownoutVoltage(BROWNOUT_VOLTAGE);

    // Drive
    leftLeader    = safeCreateMotor(LEFT_LEADER_ID,    false, "LEFT_LEADER");
    rightLeader   = safeCreateMotor(RIGHT_LEADER_ID,   true,  "RIGHT_LEADER");
    leftFollower  = safeCreateFollower(LEFT_FOLLOWER_ID,  leftLeader,  false, "LEFT_FOLLOWER");
    rightFollower = safeCreateFollower(RIGHT_FOLLOWER_ID, rightLeader, false, "RIGHT_FOLLOWER");

    // Intake / Outake
    intakeMotor = safeCreateMotor(INTAKE_ID, false, "INTAKE");
    outakeMotor = safeCreateMotor(OUTAKE_ID, false, "OUTAKE");

    // DifferentialDrive
    if (leftLeader != null && rightLeader != null) {
      drive = new DifferentialDrive(leftLeader, rightLeader);
      drive.setDeadband(DEADBAND);
      drive.setSafetyEnabled(true);
    } else {
      System.out.println("[ERREUR CRITIQUE] Moteurs drive non initialisés !");
    }

    controller = new XboxController(0);
    rawStick   = new Joystick(0);
    navX       = initGyro();

    // Charger mode vitesse sauvegardé
    try {
      String saved = Preferences.getString(PREF_SPEED_MODE, SpeedMode.TURBO.name());
      currentSpeedMode = SpeedMode.valueOf(saved);
    } catch (Exception e) {
      currentSpeedMode = SpeedMode.TURBO;
    }

    if (drive != null) drive.setMaxOutput(getSpeedValue());

    // Dashboard
    SmartDashboard.putString ("Speed Mode",         currentSpeedMode.name());
    SmartDashboard.putNumber ("Max Output (%)",      getSpeedValue() * 100);
    SmartDashboard.putBoolean("Gyro Connected",      navX != null);
    SmartDashboard.putNumber ("Heading (deg)",       0.0);
    SmartDashboard.putNumber ("Rate (deg-s)",        0.0);
    SmartDashboard.putBoolean("Rotation Alert",      false);
    SmartDashboard.putBoolean("Intake Overheat",     false);
    SmartDashboard.putBoolean("Outake Overheat",     false);
    SmartDashboard.putNumber ("Battery Voltage (V)", RobotController.getBatteryVoltage());

    printSpeedMode("INIT");

    System.out.println("============================================");
    System.out.println("[LEFT]     Leader=" + LEFT_LEADER_ID + "  Follower=" + LEFT_FOLLOWER_ID);
    System.out.println("[RIGHT]    Leader=" + RIGHT_LEADER_ID + "  Follower=" + RIGHT_FOLLOWER_ID);
    System.out.println("[INTAKE]   ID=" + INTAKE_ID + "  → Axis Z   (Z+ = 100% | Z- = -100%)");
    System.out.println("[OUTAKE]   ID=" + OUTAKE_ID + "  → Axis ZRot(ZR+= 100% | ZR-= -100%)");
    System.out.println("[DRIVE]    LY=Avant/Arrière  LX=Gauche/Droite");
    System.out.println("[BOUTON]   A = Cycle vitesse");
    System.out.println("[BROWNOUT] Seuil = " + BROWNOUT_VOLTAGE + "V");
    System.out.println("============================================");
  }

  // ===================== TELEOP INIT =====================

  @Override
  public void teleopInit() {
    speedLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
    intakeLimiter.reset(0.0);
    outakeLimiter.reset(0.0);

    if (drive != null)       drive.stopMotor();
    if (intakeMotor != null) intakeMotor.set(0.0);
    if (outakeMotor != null) outakeMotor.set(0.0);

    intakeOverheat = false;
    outakeOverheat = false;
    tiltAlerted    = false;

    printSpeedMode("TELEOP START");
    DataLogManager.log("[TELEOP] Démarrage téléop");
  }

  // ===================== TELEOP PERIODIC =====================

  @Override
  public void teleopPeriodic() {

    // ----- Bouton A : cycle vitesse (rising edge) -----
    boolean btnA = controller.getAButton();
    if (btnA && !prevA) {
      switch (currentSpeedMode) {
        case SLOW:   currentSpeedMode = SpeedMode.NORMAL; break;
        case NORMAL: currentSpeedMode = SpeedMode.TURBO;  break;
        case TURBO:  currentSpeedMode = SpeedMode.SLOW;   break;
      }
      if (drive != null) drive.setMaxOutput(getSpeedValue());
      Preferences.setString(PREF_SPEED_MODE, currentSpeedMode.name());
      printSpeedMode("CHANGEMENT");
      DataLogManager.log("[MODE] " + currentSpeedMode.name());
    }
    prevA = btnA;

    // ----- DRIVE : LY=Avant/Arrière | LX=Gauche/Droite -----
    double speed    = -applyDeadband(controller.getLeftY());
    double rotation =  applyDeadband(controller.getLeftX());

    speed    = speedLimiter.calculate(speed);
    rotation = rotationLimiter.calculate(rotation);

    if (drive != null) {
      drive.arcadeDrive(speed, rotation, false);
    }

    // ----- INTAKE : Axis Z → vitesse MAXIMUM dans le sens du joystick -----
    double zRaw = rawStick.getZ();
    // ✅ Si joystick dépasse le seuil → on force à ±INTAKE_MAX_SPEED (pas proportionnel)
    double intakeCmd = 0.0;
    if (zRaw > AXIS_THRESHOLD)       intakeCmd = +INTAKE_MAX_SPEED;  // Z+ = 100%
    else if (zRaw < -AXIS_THRESHOLD) intakeCmd = -INTAKE_MAX_SPEED;  // Z- = -100%

    // ----- OUTAKE : Axis ZRot → vitesse MAXIMUM dans le sens du joystick -----
    double zRotRaw = rawStick.getTwist();
    double outakeCmd = 0.0;
    if (zRotRaw > AXIS_THRESHOLD)       outakeCmd = +OUTAKE_MAX_SPEED;  // ZR+ = 100%
    else if (zRotRaw < -AXIS_THRESHOLD) outakeCmd = -OUTAKE_MAX_SPEED;  // ZR- = -100%

    handleIntakeOutake(intakeCmd, outakeCmd);

    updateGyro();

    // ----- Dashboard -----
    SmartDashboard.putString ("Speed Mode",          currentSpeedMode.name());
    SmartDashboard.putNumber ("Max Output (%)",       getSpeedValue() * 100);
    SmartDashboard.putNumber ("Speed Input",          speed);
    SmartDashboard.putNumber ("Rotation Input",       rotation);
    SmartDashboard.putNumber ("Axis Z  (Intake raw)", zRaw);
    SmartDashboard.putNumber ("Axis ZR (Outake raw)", zRotRaw);
    SmartDashboard.putNumber ("Intake CMD",           intakeCmd);
    SmartDashboard.putNumber ("Outake CMD",           outakeCmd);
    SmartDashboard.putBoolean("Intake ON",            intakeCmd != 0.0);
    SmartDashboard.putBoolean("Outake ON",            outakeCmd != 0.0);
    SmartDashboard.putBoolean("Intake Overheat",      intakeOverheat);
    SmartDashboard.putBoolean("Outake Overheat",      outakeOverheat);
    SmartDashboard.putNumber ("Battery Voltage (V)",  RobotController.getBatteryVoltage());

    if (leftLeader    != null) SmartDashboard.putNumber("Left  Leader  Current (A)", leftLeader.getOutputCurrent());
    if (leftFollower  != null) SmartDashboard.putNumber("Left  Follow  Current (A)", leftFollower.getOutputCurrent());
    if (rightLeader   != null) SmartDashboard.putNumber("Right Leader  Current (A)", rightLeader.getOutputCurrent());
    if (rightFollower != null) SmartDashboard.putNumber("Right Follow  Current (A)", rightFollower.getOutputCurrent());
    if (intakeMotor   != null) SmartDashboard.putNumber("Intake Current (A)",        intakeMotor.getOutputCurrent());
    if (outakeMotor   != null) SmartDashboard.putNumber("Outake Current (A)",        outakeMotor.getOutputCurrent());
  }

  // ===================== AUTONOME =====================

  @Override
  public void autonomousInit() {
    autoTimer.reset();
    autoTimer.start();
    resetGyro();
    if (intakeMotor != null) intakeMotor.set(0.0);
    if (outakeMotor != null) outakeMotor.set(0.0);
    if (drive != null)       drive.setMaxOutput(1.0);
    DataLogManager.log("[AUTO] Démarrage autonome");
  }

  @Override
  public void autonomousPeriodic() {
    if (drive == null) return;
    updateGyro();
    if (autoTimer.get() < 2.0) {
      drive.arcadeDrive(0.40, 0.0, false);
    } else {
      drive.stopMotor();
    }
  }

  // ===================== DISABLED =====================

  @Override
  public void disabledInit() {
    if (drive != null)       drive.stopMotor();
    if (intakeMotor != null) intakeMotor.set(0.0);
    if (outakeMotor != null) outakeMotor.set(0.0);
    DataLogManager.log("[ROBOT] Disabled — tous les moteurs arrêtés");
    System.out.println("[ROBOT] Disabled — tous les moteurs arrêtés");
  }

  // ===================== INTAKE / OUTAKE =====================

  private void handleIntakeOutake(double intakeCmd, double outakeCmd) {

    // Surchauffe intake
    if (intakeMotor != null) {
      double iCurrent = intakeMotor.getOutputCurrent();
      if (iCurrent > MAX_INTAKE_CURRENT && !intakeOverheat) {
        intakeOverheat = true;
        intakeMotor.set(0.0);
        String w = "[WARNING] INTAKE surchauffe ! " + String.format("%.1f", iCurrent) + "A — coupure";
        System.out.println(w);
        DataLogManager.log(w);
      } else if (iCurrent <= MAX_INTAKE_CURRENT * 0.85) {
        intakeOverheat = false;
      }
    }

    // Surchauffe outake
    if (outakeMotor != null) {
      double oCurrent = outakeMotor.getOutputCurrent();
      if (oCurrent > MAX_OUTAKE_CURRENT && !outakeOverheat) {
        outakeOverheat = true;
        outakeMotor.set(0.0);
        String w = "[WARNING] OUTAKE surchauffe ! " + String.format("%.1f", oCurrent) + "A — coupure";
        System.out.println(w);
        DataLogManager.log(w);
      } else if (oCurrent <= MAX_OUTAKE_CURRENT * 0.85) {
        outakeOverheat = false;
      }
    }

    // Priorité outake > intake — vitesse max dans le sens du joystick
    if (outakeCmd != 0.0 && !outakeOverheat) {
      if (outakeMotor != null) outakeMotor.set(outakeLimiter.calculate(outakeCmd));
      if (intakeMotor != null) { intakeLimiter.reset(0.0); intakeMotor.set(0.0); }
    } else if (intakeCmd != 0.0 && !intakeOverheat) {
      if (intakeMotor != null) intakeMotor.set(intakeLimiter.calculate(intakeCmd));
      if (outakeMotor != null) { outakeLimiter.reset(0.0); outakeMotor.set(0.0); }
    } else {
      if (intakeMotor != null) { intakeLimiter.reset(0.0); intakeMotor.set(0.0); }
      if (outakeMotor != null) { outakeLimiter.reset(0.0); outakeMotor.set(0.0); }
    }

    // Logs au changement
    boolean intakeOn = intakeCmd != 0.0;
    boolean outakeOn = outakeCmd != 0.0;

    if (intakeOn != prevIntakeOn) {
      System.out.println(intakeOn
          ? "[INTAKE] ON  @ " + (intakeCmd > 0 ? "+100% (Z+)" : "-100% (Z-)")
          : "[INTAKE] OFF");
      prevIntakeOn = intakeOn;
    }
    if (outakeOn != prevOutakeOn) {
      System.out.println(outakeOn
          ? "[OUTAKE] ON  @ " + (outakeCmd > 0 ? "+100% (ZR+)" : "-100% (ZR-)")
          : "[OUTAKE] OFF");
      prevOutakeOn = outakeOn;
    }
  }

  // ===================== VITESSE =====================

  private void printSpeedMode(String context) {
    String bar, label;
    switch (currentSpeedMode) {
      case SLOW:
        bar = "██░░░░░░░░  50%"; label = "SLOW";   break;
      case NORMAL:
        bar = "████████░░  75%"; label = "NORMAL"; break;
      case TURBO: default:
        bar = "██████████ 100%"; label = "TURBO";  break;
    }
    System.out.println("============================================");
    System.out.println("[VITESSE] " + context);
    System.out.println("  Mode   : " + label);
    System.out.println("  Output : " + bar);
    System.out.println("============================================");
  }

  // ===================== GYRO =====================

  private ADXRS450_Gyro initGyro() {
    try {
      ADXRS450_Gyro gyro = new ADXRS450_Gyro();
      gyro.calibrate();
      System.out.println("[GYRO] ADXRS450 initialisé ✓");
      return gyro;
    } catch (Exception e) {
      System.out.println("[ERREUR] Gyro non détecté : " + e.getMessage());
      return null;
    }
  }

  private void updateGyro() {
    if (navX == null) return;
    double heading   = navX.getAngle();
    double rate      = navX.getRate();
    boolean fastSpin = Math.abs(rate) > 300.0;

    if (fastSpin && !tiltAlerted) {
      String w = "[WARNING] Rotation rapide ! Rate=" + String.format("%.1f", rate) + " deg/s";
      System.out.println(w);
      DataLogManager.log(w);
      tiltAlerted = true;
    } else if (!fastSpin) {
      tiltAlerted = false;
    }

    SmartDashboard.putBoolean("Gyro Connected", true);
    SmartDashboard.putNumber ("Heading (deg)",  heading);
    SmartDashboard.putNumber ("Rate (deg-s)",   rate);
    SmartDashboard.putBoolean("Rotation Alert", fastSpin);
  }

  private void resetGyro() {
    if (navX == null) return;
    navX.reset();
    System.out.println("[GYRO] Cap remis à zéro");
  }

  // ===================== UTILITAIRES =====================

  private SparkMax safeCreateMotor(int id, boolean inverted, String label) {
    try {
      SparkMax motor = new SparkMax(id, MOTOR_TYPE);
      SparkMaxConfig config = new SparkMaxConfig();
      config.smartCurrentLimit(CURRENT_LIMIT);
      config.idleMode(IdleMode.kBrake);
      config.inverted(inverted);
      config.voltageCompensation(12.0);
      config.openLoopRampRate(0.10);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      System.out.println("[OK] Moteur " + label + " (ID=" + id + ") initialisé");
      return motor;
    } catch (Exception e) {
      System.out.println("[ERREUR] Moteur " + label + " (ID=" + id + ") : " + e.getMessage());
      DataLogManager.log("[ERREUR] Moteur " + label + " ID=" + id);
      return null;
    }
  }

  private SparkMax safeCreateFollower(int id, SparkMax leader, boolean invertedVsLeader, String label) {
    try {
      SparkMax follower = new SparkMax(id, MOTOR_TYPE);
      SparkMaxConfig config = new SparkMaxConfig();
      config.smartCurrentLimit(CURRENT_LIMIT);
      config.idleMode(IdleMode.kBrake);
      config.voltageCompensation(12.0);
      config.follow(leader, invertedVsLeader);
      follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      System.out.println("[OK] Follower " + label + " (ID=" + id + ") → suit leader");
      return follower;
    } catch (Exception e) {
      System.out.println("[ERREUR] Follower " + label + " (ID=" + id + ") : " + e.getMessage());
      return null;
    }
  }

  private double applyDeadband(double value) {
    return (Math.abs(value) < DEADBAND) ? 0.0 : value;
  }

  private double getSpeedValue() {
    switch (currentSpeedMode) {
      case SLOW:   return SLOW_SPEED;
      case NORMAL: return NORMAL_SPEED;
      case TURBO:  return TURBO_SPEED;
      default:     return 1.0;
    }
  }
}
