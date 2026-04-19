package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private final Timer autoTimer = new Timer();

    
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log("[ROBOT] DataLog démarré");

        RobotController.setBrownoutVoltage(Constants.RobotConstants.BROWNOUT_VOLTAGE);

        robotContainer = new RobotContainer();

        System.out.println("============================================");
        System.out.println("[ROBOT] Initialisation terminée");
        System.out.println("============================================");
    }

    @Override
    public void teleopInit() {
        robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.teleopPeriodic();
    }

    @Override
    public void autonomousInit() {
        autoTimer.reset();
        autoTimer.start();
        robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        if (autoTimer.get() < 2.0) {
            robotContainer.getDriveSubsystem().driveArcade(0.40, 0.0);
        } else {
            robotContainer.getDriveSubsystem().stop();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.disabledInit();
        DataLogManager.log("[ROBOT] Disabled — tous les moteurs arrêtés");
        System.out.println("[ROBOT] Disabled — tous les moteurs arrêtés");
    }
}