package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class IntakeSubsystem {

    private final SparkMax intakeMotor;
    private final SparkMax outakeMotor;

    private boolean intakeOverheat = false;
    private boolean outakeOverheat = false;

    private boolean prevIntakeOn = false;
    private boolean prevOutakeOn = false;

    public IntakeSubsystem() {
        intakeMotor = createMotor(Constants.IntakeConstants.INTAKE_ID, true, "INTAKE");
        outakeMotor = createMotor(Constants.IntakeConstants.OUTAKE_ID, true, "OUTAKE");
    }

    public void runIntake(double speed) {
        checkIntakeOverheat();

        if (!intakeOverheat) {
            intakeMotor.set(speed);
        } else {
            intakeMotor.set(0.0);
        }

        logIntakeState(speed);
    }

    public void runOutake(double speed) {
        checkOutakeOverheat();

        if (!outakeOverheat) {
            outakeMotor.set(speed);
        } else {
            outakeMotor.set(0.0);
        }

        logOutakeState(speed);
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
        logIntakeState(0.0);
    }

    public void stopOutake() {
        outakeMotor.set(0.0);
        logOutakeState(0.0);
    }

    public void stopAll() {
        stopIntake();
        stopOutake();
    }

    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public double getOutakeCurrent() {
        return outakeMotor.getOutputCurrent();
    }

    public boolean isIntakeOverheat() {
        return intakeOverheat;
    }

    public boolean isOutakeOverheat() {
        return outakeOverheat;
    }

    public void resetStates() {
        intakeOverheat = false;
        outakeOverheat = false;
        prevIntakeOn = false;
        prevOutakeOn = false;
    }

    private void checkIntakeOverheat() {
        double current = intakeMotor.getOutputCurrent();

        if (current > Constants.IntakeConstants.MAX_INTAKE_CURRENT && !intakeOverheat) {
            intakeOverheat = true;
            intakeMotor.set(0.0);

            String msg = "[WARNING] INTAKE surchauffe ! " + String.format("%.1f", current) + "A — coupure";
            System.out.println(msg);
            DataLogManager.log(msg);
        } else if (current <= Constants.IntakeConstants.MAX_INTAKE_CURRENT * 0.85) {
            intakeOverheat = false;
        }
    }

    private void checkOutakeOverheat() {
        double current = outakeMotor.getOutputCurrent();

        if (current > Constants.IntakeConstants.MAX_OUTAKE_CURRENT && !outakeOverheat) {
            outakeOverheat = true;
            outakeMotor.set(0.0);

            String msg = "[WARNING] OUTAKE surchauffe ! " + String.format("%.1f", current) + "A — coupure";
            System.out.println(msg);
            DataLogManager.log(msg);
        } else if (current <= Constants.IntakeConstants.MAX_OUTAKE_CURRENT * 0.85) {
            outakeOverheat = false;
        }
    }

    private SparkMax createMotor(int id, boolean inverted, String label) {
        try {
            SparkMax motor = new SparkMax(id, Constants.IntakeConstants.MOTOR_TYPE);

            SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT);
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

    private void logIntakeState(double cmd) {
        boolean on = cmd != 0.0;

        if (on != prevIntakeOn) {
            System.out.println(on
                    ? "[INTAKE] ON @ " + (cmd > 0 ? "+100%" : "-100%")
                    : "[INTAKE] OFF");
            prevIntakeOn = on;
        }
    }

    private void logOutakeState(double cmd) {
        boolean on = cmd != 0.0;

        if (on != prevOutakeOn) {
            System.out.println(on
                    ? "[OUTAKE] ON @ " + (cmd > 0 ? "+100%" : "-100%")
                    : "[OUTAKE] OFF");
            prevOutakeOn = on;
        }
    }
}