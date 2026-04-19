package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public final class Constants {

    private Constants() {}

    public static final class DriveConstants {
        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_LEADER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;

        public static final MotorType MOTOR_TYPE = MotorType.kBrushed;
        public static final int CURRENT_LIMIT = 35;

        public static final double DEADBAND = 0.08;
        public static final double SLOW_SPEED = 0.40;
        public static final double NORMAL_SPEED = 0.75;
        public static final double TURBO_SPEED = 1.00;

        public static final double SLEW_SPEED = 3.5;
        public static final double SLEW_ROTATION = 4.5;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_ID = 5;
        public static final int OUTAKE_ID = 6;

        public static final MotorType MOTOR_TYPE = MotorType.kBrushed;
        public static final int CURRENT_LIMIT = 35;

        public static final double MAX_INTAKE_CURRENT = 30.0;
        public static final double MAX_OUTAKE_CURRENT = 30.0;

        public static final double INTAKE_MAX_SPEED = 1.0;
        public static final double OUTAKE_MAX_SPEED = 1.0;

        public static final double AXIS_THRESHOLD = 0.10;
    }

    public static final class RobotConstants {
        public static final double BROWNOUT_VOLTAGE = 7.0;
        public static final String PREF_SPEED_MODE = "speedMode";
        public static final double FAST_SPIN_THRESHOLD = 300.0;
    }

    public static final class OIConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int JOYSTICK_PORT = 1;
    }

    public static final class VisionConstants {
        public static final String CAMERA_NAME = "FrontCam";
        
        // Position de la caméra par rapport au centre du robot
        // A MESURER EN VRAI sur votre robot
        public static final double CAMERA_FORWARD_METERS = 0.20; // caméra 20 cm devant le centre
        public static final double CAMERA_SIDE_METERS = 0.0;     // centrée gauche/droite
        public static final double CAMERA_HEIGHT_METERS = 0.35;  // caméra à 35 cm du sol

    // Rotation de la caméra
        public static final double CAMERA_ROLL_RADIANS = 0.0;
        public static final double CAMERA_PITCH_RADIANS = Math.toRadians(0.0); // inclinaison haut/bas
        public static final double CAMERA_YAW_RADIANS = 0.0;
    }
}
