package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain implements Subsystem {

    private static TalonSRX 
            leftMotorA = new TalonSRX(Constants.DrivetrainConstants.leftMotorA),
            leftMotorB = new TalonSRX(Constants.DrivetrainConstants.leftMotorB), 
            rightMotorA = new TalonSRX(Constants.DrivetrainConstants.rightMotorA),
            rightMotorB = new TalonSRX(Constants.DrivetrainConstants.rightMotorB);
    private static VictorSPX 
            leftMotorC = new VictorSPX(Constants.DrivetrainConstants.leftMotorC),
            rightMotorC = new VictorSPX(Constants.DrivetrainConstants.rightMotorC);

    private static Drivetrain instance;
    public static Drivetrain getInstance(){
        if(instance == null) instance = new Drivetrain();
        return instance;
    }

    public static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
    public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DrivetrainConstants.trackwidth);
    public static DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(RobotContainer.navX.getAngle()));

    public static PIDController leftPIDController = new PIDController(DrivetrainConstants.kPVelocity, 0, 0);
    public static PIDController rightPIDController = new PIDController(DrivetrainConstants.kPVelocity, 0, 0);

    private static double lastLeft = 0;
    private static double lastRight = 0;

    private Drivetrain() {

        /* Setting follower and master speed controllers */
        leftMotorB.follow(leftMotorA);
        leftMotorC.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);
        rightMotorC.follow(rightMotorA);

        /* Inversion on opposite sides of the drivetrain */
        Arrays.asList(leftMotorA, leftMotorB, leftMotorC).forEach(motor -> motor.setInverted(true));
        Arrays.asList(rightMotorA, rightMotorB, rightMotorC).forEach(motor -> motor.setInverted(false));

        /* Common Motor Settings */
        Arrays.asList(leftMotorA, leftMotorB, leftMotorC, rightMotorA, rightMotorB, rightMotorC).forEach(motor -> {
            if(motor instanceof TalonSRX) {
                TalonSRX talon = (TalonSRX) motor;
                talon.configPeakCurrentLimit(45);
                talon.configPeakCurrentDuration(125);
                talon.configContinuousCurrentLimit(38);
                talon.enableCurrentLimit(true);
            }

            motor.configVoltageCompSaturation(12, 10);
            motor.enableVoltageCompensation(true);
            motor.configClosedloopRamp(0.05, 0);
            motor.setNeutralMode(NeutralMode.Brake);
        });

        /* Encoder settings */
        Arrays.asList(leftMotorA, rightMotorA).forEach(motor -> {
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1, 10);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        });
        leftMotorA.setSensorPhase(false);
        rightMotorA.setSensorPhase(false);

        register();

    }  

    /**
     * This function is periodically called when the subsystem is initialized
     */
    @Override
    public void periodic() {

        /* TODO: Dashboard input and displaying */

        /* Updating odometry */
        double velocityLeft = DifferentialDrive.TicksPerDecisecondtoMPS(Drivetrain.getLeftEncVelocity());
        double velocityRight = DifferentialDrive.TicksPerDecisecondtoMPS(Drivetrain.getRightEncVelocity());

        SmartDashboard.putNumber("Left Actual Velocity", velocityLeft);
        SmartDashboard.putNumber("Right Actual Velocity", velocityRight);

        SmartDashboard.putNumber("Left Error Velocity", leftPIDController.getVelocityError());
        SmartDashboard.putNumber("Right Error Velocity", rightPIDController.getVelocityError());

        SmartDashboard.putNumber("Left Target Velocity", leftPIDController.getSetpoint());
        SmartDashboard.putNumber("Right Target Velocity", rightPIDController.getSetpoint());
        
        odometry.update(Rotation2d.fromDegrees(-RobotContainer.navX.getAngle()),
                        DifferentialDrive.TicksToMeters(Drivetrain.getLeftEnc()), 
                        DifferentialDrive.TicksToMeters(Drivetrain.getRightEnc()));
        RobotContainer.falcondash.putOdom(meterToFeet(odometry.getPoseMeters()));  
    }

    /**
     * Sets odometry (used to set the position and angle offset at the beginning of
     * autonomous)
     * 
     * @param poseMeters Global pose of robot
     * @param gyroAngle  Angle robot is facing
     */
    public void setOdometry(Pose2d poseMeters, Rotation2d gyroAngle) {
        odometry.resetPosition(poseMeters, gyroAngle);
    }

    /**
     * Sets drivetrain speeds in open loop (% of max voltage)
     * 
     * @param left   Percent output of motors on left side of drivetrain
     * @param right  Percent output of motors on right side of drivetrain
     * @param isAuto Whether this is being run in autonomous
     */
    public static void setOpenLoop(Double left, Double right, boolean isAuto) {

        if(isAuto) {
            leftMotorA.set(ControlMode.PercentOutput, left/12.0);
            rightMotorA.set(ControlMode.PercentOutput, right/12.0);
        } else {
            leftMotorA.set(ControlMode.PercentOutput, left);
            rightMotorA.set(ControlMode.PercentOutput, right);
        }

        SmartDashboard.putNumber("l volt", left);
        SmartDashboard.putNumber("r volt", right);
    }

    /**
     * Sets drivetrain speeds in closed loop (m/s)
     * 
     * @param left   Velocity to set left side of drivetrain to
     * @param right  Velocity to set right side of drivetrain to
     */
    public static void setClosedLoop(Double left, Double right) {
        // Deriving acceleration from velocities, assuming loop time of 0.02 seconds (50ms)
        double accelLeft = (left - lastLeft) / 0.02;
        double accelRight = (right - lastRight) / 0.02;

        // Setting previous velocities for next loop's acceleration calculation
        lastLeft = left;
        lastRight = right;

        // Calculating feedforwards from velocity and acceleration 
        double feedforwardLeft = feedforward.calculate(left, accelLeft) / 12;
        double feedforwardRight = feedforward.calculate(right, accelRight) /12;

        // Converting left and right meters/sec to Talon native units
        left = DifferentialDrive.MPStoTicksPerDecisecond(left);
        right = DifferentialDrive.MPStoTicksPerDecisecond(right);

        leftMotorA.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, feedforwardLeft);
        rightMotorA.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, feedforwardRight);
    }

    // Clearing previous velocity values, transition from autonomous to teleop
    public static void clearLastVelocities(){
        lastLeft = 0;
        lastRight = 0;
    }

    // Zeroes encoders 
    public void resetEncoders() {
        Arrays.asList(leftMotorA, rightMotorA).forEach(motor -> motor.setSelectedSensorPosition(0));
    }

    public void resetEncoders(int left, int right) {
        rightMotorA.setSelectedSensorPosition(right);
        leftMotorA.setSelectedSensorPosition(left);
    }

    public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(DifferentialDrive.TicksPerDecisecondtoMPS(getLeftEncVelocity()), DifferentialDrive.TicksPerDecisecondtoMPS(getRightEncVelocity()));
    }

    // Returns the current measurement of the left drivetrain encoder
    public static double getLeftEnc() {
        return leftMotorA.getSelectedSensorPosition();
    }

    // Returns the current measurement of the right drivetrain encoder
    public static double getRightEnc() {
        return rightMotorA.getSelectedSensorPosition();
    }

    // Returns the current velocity measurement of the left drivetrain encoder
    public static double getLeftEncVelocity() {
        return leftMotorA.getSelectedSensorVelocity();
    }

    // Returns the current velocity measurement of the right drivetrain encoder
    public static double getRightEncVelocity() {
        return rightMotorA.getSelectedSensorVelocity();
    }

    /* Static class to handle conversion from joystick inputs into left and right percent speeds */
    public static class DifferentialDrive {
        private static double quickStopAccumulator = 0.0;

        public static WheelState curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
            final double angularPower;
            final boolean overPower;

            if (isQuickTurn) {
                if (Math.abs(linearPercent) < Constants.DrivetrainConstants.kQuickStopThreshold) {
                    quickStopAccumulator = (1 - Constants.DrivetrainConstants.kQuickStopAlpha) * quickStopAccumulator
                            + Constants.DrivetrainConstants.kQuickStopAlpha * clamp(curvaturePercent, -1.0, 1.0) * 2.0;
                }
                overPower = true;
                angularPower = curvaturePercent;

            } else {
                overPower = false;
                angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

                if (quickStopAccumulator > 1) {
                    quickStopAccumulator -= 1.0;
                } else if (quickStopAccumulator < -1) {
                    quickStopAccumulator += 1.0;
                } else {
                    quickStopAccumulator = 0.0;
                }
            }

            double left = linearPercent + angularPower;
            double right = linearPercent - angularPower;

            // If rotation is overpowered, reduce both outputs to within acceptable range
            if (overPower) {
                if (left > 1) {
                    right -= left - 1;
                    left = 1;
                } else if (right > 1) {
                    left -= right - 1;
                    right = 1;
                } else if (left < -1) {
                    right -= left + 1;
                    left = -1;
                } else if (right < -1) {
                    left -= right + 1;
                    right = -1;
                }
            }

            // Normalize the wheel speeds
            double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
            if (maxMagnitude > 1) {
                left /= maxMagnitude;
                right /= maxMagnitude;
            }

            return new WheelState(left, right);
        }

        private static double clamp(double val, double low, double high) {
            return Math.max(low, Math.min(high, val));
        }

        public static double FPStoTicksPerDecisecond(double val) {
            return val * DrivetrainConstants.ticksPerRotation * 12 / (10 * 4 * Math.PI);
        }

        public static double MPStoTicksPerDecisecond(double val) {
            return val * DrivetrainConstants.ticksPerRotation / (10 * Math.PI * DrivetrainConstants.wheelDiameter);
        }

        public static double TicksPerDecisecondtoFPS(double val) {
            return val * 10 * 4 * Math.PI / (DrivetrainConstants.ticksPerRotation * 12);
        }

        public static double TicksPerDecisecondtoMPS(double val) {
            return val * 10 * DrivetrainConstants.wheelDiameter * Math.PI / DrivetrainConstants.ticksPerRotation;
        }

        public static double TicksToMeters(double val) {
            return val * 0.319185814 / DrivetrainConstants.ticksPerRotation;
        }
    }

    /* Static class to contain the speeds of each side of the drivetrain */
    public static class WheelState {
        public double left, right;

        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static Pose2d meterToFeet(Pose2d meter) {
        Translation2d feetTranslation = meter.getTranslation().div(1/3.28084);
        return new Pose2d(feetTranslation, meter.getRotation());
    }
}