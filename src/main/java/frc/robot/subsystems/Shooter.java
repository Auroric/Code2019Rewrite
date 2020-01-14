package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    private static TalonSRX 
        motorLow = new TalonSRX(ShooterConstants.lowerMotor),
        motorHigh = new TalonSRX(ShooterConstants.upperMotor);

    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    public static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

    private static double lastRPMTop = 0;
    private static double lastRPMBottom = 0;

    private Shooter() {
        Arrays.asList(motorLow, motorHigh).forEach(motor -> {
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

        motorLow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        motorLow.setSensorPhase(true);

        motorHigh.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        motorHigh.setSensorPhase(false);
    }

    /**
     * Sets shooter speeds in percent of max
     * 
     * @param top
     * @param bottom
     */
    public static void setOpenLoop(Double top, Double bottom) {
        motorHigh.set(ControlMode.PercentOutput, top);
        motorLow.set(ControlMode.PercentOutput, bottom);
    }

    /**
     * Sets shooter speeds in RPM
     * 
     * @param top     RPM of the top shooter shaft
     * @param bottom  RPM of the bottom shooter shaft
     */
    public static void setClosedLoop(Double top, Double bottom) {
        double accelTop = (top - lastRPMTop) / 0.02;
        double accelBottom = (bottom - lastRPMBottom) / 0.02;

        lastRPMTop = top;
        lastRPMBottom = bottom;

        double feedforwardTop = feedforward.calculate(top, accelTop) / 12;
        double feedforwardBottom = feedforward.calculate(bottom, accelBottom) / 12;
        
        motorHigh.set(ControlMode.Velocity, top, DemandType.ArbitraryFeedForward, feedforwardTop);
        motorLow.set(ControlMode.Velocity, bottom, DemandType.ArbitraryFeedForward, feedforwardBottom);
    }

    // Clearing previous velocity values, transition from autonomous to teleop
    public static void clearLastVelocities(){
        lastRPMTop = 0;
        lastRPMBottom = 0;
    }

    // Zeroes encoders 
    public void resetEncoders() {
        Arrays.asList(motorHigh, motorLow).forEach(motor -> motor.setSelectedSensorPosition(0));
    }

}