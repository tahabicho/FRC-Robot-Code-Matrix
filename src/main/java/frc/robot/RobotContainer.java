package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final XboxController controller =
            new XboxController(Constants.OIConstants.CONTROLLER_PORT);
    private final Joystick rawStick =
            new Joystick(Constants.OIConstants.JOYSTICK_PORT);

    private ADXRS450_Gyro gyro;
    private boolean prevA = false;
    private boolean rotationAlerted = false;

    public RobotContainer() {
        gyro = initGyro();
        initializeDashboard();
        printMappings();
    }

    public void teleopInit() {
        driveSubsystem.resetLimiters();
        driveSubsystem.stop();
        intakeSubsystem.stopAll();
        intakeSubsystem.resetStates();

        prevA = false;
        rotationAlerted = false;

        driveSubsystem.printSpeedMode("TELEOP START");
        DataLogManager.log("[TELEOP] Démarrage téléop");
    }

    public void teleopPeriodic() {
        handleSpeedButton();
        handleDrive();
        handleIntakeOutake();
        updateGyro();
        updateDashboard();
    }

    public void autonomousInit() {
        resetGyro();
        intakeSubsystem.stopAll();
        driveSubsystem.setMaxOutput(1.0);
        DataLogManager.log("[AUTO] Démarrage autonome");
    }

    public void disabledInit() {
        driveSubsystem.stop();
        intakeSubsystem.stopAll();
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    private void handleSpeedButton() {
        boolean btnA = controller.getAButton();

        if (btnA && !prevA) {
            driveSubsystem.cycleSpeedMode();
        }

        prevA = btnA;
    }

    private void handleDrive() {
        double speed = -controller.getLeftY();
        double rotation = controller.getLeftX();

        driveSubsystem.driveArcade(speed, rotation);

        SmartDashboard.putNumber("Speed Input", speed);
        SmartDashboard.putNumber("Rotation Input", rotation);
    }

    private void handleIntakeOutake() {
        double zRaw = rawStick.getZ();
        double zRotRaw = rawStick.getTwist();

        double intakeCmd = 0.0;
        if (zRaw > Constants.IntakeConstants.AXIS_THRESHOLD) {
            intakeCmd = Constants.IntakeConstants.INTAKE_MAX_SPEED;
        } else if (zRaw < -Constants.IntakeConstants.AXIS_THRESHOLD) {
            intakeCmd = -Constants.IntakeConstants.INTAKE_MAX_SPEED;
        }

        double outakeCmd = 0.0;
        if (Math.abs(zRotRaw) > Constants.IntakeConstants.AXIS_THRESHOLD) {
            outakeCmd = Constants.IntakeConstants.OUTAKE_MAX_SPEED;
        }

        boolean leftBumper = controller.getLeftBumper();
        boolean rightBumper = controller.getRightBumper();

        if (leftBumper) {
            intakeCmd = -Constants.IntakeConstants.INTAKE_MAX_SPEED;
        }

        if (rightBumper) {
            outakeCmd = Constants.IntakeConstants.OUTAKE_MAX_SPEED;
        }

        if (intakeCmd != 0.0) {
            intakeSubsystem.runIntake(intakeCmd);
        } else {
            intakeSubsystem.stopIntake();
        }

        if (outakeCmd != 0.0) {
            intakeSubsystem.runOutake(outakeCmd);
        } else {
            intakeSubsystem.stopOutake();
        }

        SmartDashboard.putNumber("Axis Z (Intake raw)", zRaw);
        SmartDashboard.putNumber("Axis ZR (Outake raw)", zRotRaw);
        SmartDashboard.putNumber("Intake CMD", intakeCmd);
        SmartDashboard.putNumber("Outake CMD", outakeCmd);
        SmartDashboard.putBoolean("Intake ON", intakeCmd != 0.0);
        SmartDashboard.putBoolean("Outake ON", outakeCmd != 0.0);
    }

    private ADXRS450_Gyro initGyro() {
        try {
            ADXRS450_Gyro g = new ADXRS450_Gyro();
            g.calibrate();
            System.out.println("[GYRO] ADXRS450 initialisé ✓");
            return g;
        } catch (Exception e) {
            System.out.println("[ERREUR] Gyro non détecté : " + e.getMessage());
            return null;
        }
    }

    private void updateGyro() {
        if (gyro == null) {
            SmartDashboard.putBoolean("Gyro Connected", false);
            return;
        }

        double heading = gyro.getAngle();
        double rate = gyro.getRate();
        boolean fastSpin = Math.abs(rate) > Constants.RobotConstants.FAST_SPIN_THRESHOLD;

        if (fastSpin && !rotationAlerted) {
            String msg = "[WARNING] Rotation rapide ! Rate=" + String.format("%.1f", rate) + " deg/s";
            System.out.println(msg);
            DataLogManager.log(msg);
            rotationAlerted = true;
        } else if (!fastSpin) {
            rotationAlerted = false;
        }

        SmartDashboard.putBoolean("Gyro Connected", true);
        SmartDashboard.putNumber("Heading (deg)", heading);
        SmartDashboard.putNumber("Rate (deg-s)", rate);
        SmartDashboard.putBoolean("Rotation Alert", fastSpin);
    }

    private void resetGyro() {
        if (gyro != null) {
            gyro.reset();
            System.out.println("[GYRO] Cap remis à zéro");
        }
    }

    private void initializeDashboard() {
        SmartDashboard.putString("Speed Mode", driveSubsystem.getSpeedModeName());
        SmartDashboard.putNumber("Max Output (%)", driveSubsystem.getSpeedValue() * 100);
        SmartDashboard.putBoolean("Gyro Connected", gyro != null);
        SmartDashboard.putNumber("Heading (deg)", 0.0);
        SmartDashboard.putNumber("Rate (deg-s)", 0.0);
        SmartDashboard.putBoolean("Rotation Alert", false);
        SmartDashboard.putBoolean("Intake Overheat", false);
        SmartDashboard.putBoolean("Outake Overheat", false);
        SmartDashboard.putNumber("Battery Voltage (V)", RobotController.getBatteryVoltage());
    }

    private void updateDashboard() {
        SmartDashboard.putString("Speed Mode", driveSubsystem.getSpeedModeName());
        SmartDashboard.putNumber("Max Output (%)", driveSubsystem.getSpeedValue() * 100);
        SmartDashboard.putNumber("Battery Voltage (V)", RobotController.getBatteryVoltage());

        SmartDashboard.putNumber("Left Leader Current (A)", driveSubsystem.getLeftLeaderCurrent());
        SmartDashboard.putNumber("Left Follower Current (A)", driveSubsystem.getLeftFollowerCurrent());
        SmartDashboard.putNumber("Right Leader Current (A)", driveSubsystem.getRightLeaderCurrent());
        SmartDashboard.putNumber("Right Follower Current (A)", driveSubsystem.getRightFollowerCurrent());

        SmartDashboard.putNumber("Intake Current (A)", intakeSubsystem.getIntakeCurrent());
        SmartDashboard.putNumber("Outake Current (A)", intakeSubsystem.getOutakeCurrent());

        SmartDashboard.putBoolean("Intake Overheat", intakeSubsystem.isIntakeOverheat());
        SmartDashboard.putBoolean("Outake Overheat", intakeSubsystem.isOutakeOverheat());
    }

    private void printMappings() {
        System.out.println("============================================");
        System.out.println("[LEFT]     Leader=" + Constants.DriveConstants.LEFT_LEADER_ID
                + "  Follower=" + Constants.DriveConstants.LEFT_FOLLOWER_ID);
        System.out.println("[RIGHT]    Leader=" + Constants.DriveConstants.RIGHT_LEADER_ID
                + "  Follower=" + Constants.DriveConstants.RIGHT_FOLLOWER_ID);
        System.out.println("[INTAKE]   ID=" + Constants.IntakeConstants.INTAKE_ID
                + "  → Axis Z");
        System.out.println("[OUTAKE]   ID=" + Constants.IntakeConstants.OUTAKE_ID
                + "  → Axis Twist");
        System.out.println("[DRIVE]    LY=Avant/Arrière  LX=Gauche/Droite");
        System.out.println("[BOUTON]   A = Cycle vitesse");
        System.out.println("[BUMPER]   LB = Intake inverse | RB = Outake normal");
        System.out.println("[BROWNOUT] Seuil = " + Constants.RobotConstants.BROWNOUT_VOLTAGE + "V");
        System.out.println("============================================");
    }
}