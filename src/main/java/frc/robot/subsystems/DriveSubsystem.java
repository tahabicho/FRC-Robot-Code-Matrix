package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class DriveSubsystem {

    public enum SpeedMode { SLOW, NORMAL, TURBO }

    private final SparkMax leftLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightLeader;
    private final SparkMax rightFollower;

    private final DifferentialDrive drive;

    private final SlewRateLimiter speedLimiter =
            new SlewRateLimiter(Constants.DriveConstants.SLEW_SPEED);
    private final SlewRateLimiter rotationLimiter =
            new SlewRateLimiter(Constants.DriveConstants.SLEW_ROTATION);

    private SpeedMode currentSpeedMode = SpeedMode.TURBO;

    public DriveSubsystem() {
        leftLeader = createMotor(Constants.DriveConstants.LEFT_LEADER_ID, false, "LEFT_LEADER");
        rightLeader = createMotor(Constants.DriveConstants.RIGHT_LEADER_ID, true, "RIGHT_LEADER");

        leftFollower = createFollower(
                Constants.DriveConstants.LEFT_FOLLOWER_ID, leftLeader, false, "LEFT_FOLLOWER");
        rightFollower = createFollower(
                Constants.DriveConstants.RIGHT_FOLLOWER_ID, rightLeader, false, "RIGHT_FOLLOWER");

        drive = new DifferentialDrive(leftLeader, rightLeader);
        drive.setDeadband(Constants.DriveConstants.DEADBAND);
        drive.setSafetyEnabled(true);

        loadSavedSpeedMode();
        drive.setMaxOutput(getSpeedValue());
    }

    public void resetLimiters() {
        speedLimiter.reset(0.0);
        rotationLimiter.reset(0.0);
    }

    public void driveArcade(double speed, double rotation) {
        speed = applyDeadband(speed);
        rotation = applyDeadband(rotation);

        speed = speedLimiter.calculate(speed);
        rotation = rotationLimiter.calculate(rotation);

        drive.arcadeDrive(speed, rotation, false);
    }

    public void stop() {
        drive.stopMotor();
    }

    public void setMaxOutput(double value) {
        drive.setMaxOutput(value);
    }

    public void cycleSpeedMode() {
        switch (currentSpeedMode) {
            case SLOW:
                currentSpeedMode = SpeedMode.NORMAL;
                break;
            case NORMAL:
                currentSpeedMode = SpeedMode.TURBO;
                break;
            case TURBO:
            default:
                currentSpeedMode = SpeedMode.SLOW;
                break;
        }

        drive.setMaxOutput(getSpeedValue());
        Preferences.setString(Constants.RobotConstants.PREF_SPEED_MODE, currentSpeedMode.name());

        printSpeedMode("CHANGEMENT");
        DataLogManager.log("[MODE] " + currentSpeedMode.name());
    }

    public String getSpeedModeName() {
        return currentSpeedMode.name();
    }

    public double getSpeedValue() {
        switch (currentSpeedMode) {
            case SLOW:
                return Constants.DriveConstants.SLOW_SPEED;
            case NORMAL:
                return Constants.DriveConstants.NORMAL_SPEED;
            case TURBO:
            default:
                return Constants.DriveConstants.TURBO_SPEED;
        }
    }

    public double getLeftLeaderCurrent() {
        return leftLeader.getOutputCurrent();
    }

    public double getLeftFollowerCurrent() {
        return leftFollower.getOutputCurrent();
    }

    public double getRightLeaderCurrent() {
        return rightLeader.getOutputCurrent();
    }

    public double getRightFollowerCurrent() {
        return rightFollower.getOutputCurrent();
    }

    private void loadSavedSpeedMode() {
        try {
            String saved = Preferences.getString(
                    Constants.RobotConstants.PREF_SPEED_MODE,
                    SpeedMode.TURBO.name()
            );
            currentSpeedMode = SpeedMode.valueOf(saved);
        } catch (Exception e) {
            currentSpeedMode = SpeedMode.TURBO;
        }
    }

    private SparkMax createMotor(int id, boolean inverted, String label) {
        try {
            SparkMax motor = new SparkMax(id, Constants.DriveConstants.MOTOR_TYPE);

            SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(Constants.DriveConstants.CURRENT_LIMIT);
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
            throw e;
        }
    }

    private SparkMax createFollower(int id, SparkMax leader, boolean invertedVsLeader, String label) {
        try {
            SparkMax follower = new SparkMax(id, Constants.DriveConstants.MOTOR_TYPE);

            SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(Constants.DriveConstants.CURRENT_LIMIT);
            config.idleMode(IdleMode.kBrake);
            config.voltageCompensation(12.0);
            config.follow(leader, invertedVsLeader);

            follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            System.out.println("[OK] Follower " + label + " (ID=" + id + ") initialisé");
            return follower;
        } catch (Exception e) {
            System.out.println("[ERREUR] Follower " + label + " (ID=" + id + ") : " + e.getMessage());
            DataLogManager.log("[ERREUR] Follower " + label + " ID=" + id);
            throw e;
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < Constants.DriveConstants.DEADBAND ? 0.0 : value;
    }

    public void printSpeedMode(String context) {
        String bar;
        String label;

        switch (currentSpeedMode) {
            case SLOW:
                bar = "██░░░░░░░░  40%";
                label = "SLOW";
                break;
            case NORMAL:
                bar = "███████░░░  75%";
                label = "NORMAL";
                break;
            case TURBO:
            default:
                bar = "██████████ 100%";
                label = "TURBO";
                break;
        }

        System.out.println("============================================");
        System.out.println("[VITESSE] " + context);
        System.out.println("  Mode   : " + label);
        System.out.println("  Output : " + bar);
        System.out.println("============================================");
    }
}