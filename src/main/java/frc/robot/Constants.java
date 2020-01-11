package frc.robot;

public class Constants {

    /* Differential Drive Settings */
    public static class DriverConstants {

        public static final double kJoystickDeadband = 0.03; // How much of joystick is "dead" zone [0,1]
        public static final double kTriggerDeadband = 0.05; // How much of trigger is "dead" zone [0,1]

        public static final double kTurnSens = 1; // Maximum normal turning rate (in percent of max) to allow robot to turn to [0,1]
        public static final double kDriveSens = 1; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 1; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to [0,1]
    
    }

    /* Vision PD Controller Gains */
    public static class VisionConstants {

        public static final int kPAim = 0;
        public static final int kDAim = 0;

        public static final int kPDist = 0;
        public static final int kDDist = 0;

        public static final int acceptablePixelError = 0; // Acceptable error for x and y-axes (in camera pixels)
        public static final double acceptableDistAdjustError = 0.05; // Minimum speed value to curvature drive instead of turn in place to align [0,1]

    }

    /* Cheesydrive Settings */
    public class DrivetrainConstants {

        /* Auto Constants */
        public static final double kS = 1.32; // Minimum voltage required to overcome static friction (V)
        public static final double kV = 1.94; // Speed per volt (volt seconds per meter)
        public static final double kA = 0.67; // Acceleration per volt (volt seconds squared per meter)

        public static final double kPVelocity = 6.81;

        public static final double kTopSpeedMPS = 5.41134109818;
        public static final double kQuickStopThreshold = 0.2;
        public static final double kQuickStopAlpha = 0.1;

        public static final double trackwidth = 0.5207; // Wheelbase in meters
        public static final double ticksPerRotation = 1024;
        public static final double wheelRadius = 0.1016; // Wheel radius in meters

        /* Drivetrain Motor IDs */
        public static final int leftMotorA = 1; // TalonSRX
        public static final int leftMotorB = 2; // TalonSRX
        public static final int leftMotorC = 1; // VictorSPX

        public static final int rightMotorA = 3; // TalonSRX
        public static final int rightMotorB = 4; // TalonSRX
        public static final int rightMotorC = 2; // VictorSPX


    }

    /* Misc Settings */
    public static class ComponentConstants {

        public static final int kPCM_ID = 1;
        public static final int kCompressor_ID = 0;

    }
}