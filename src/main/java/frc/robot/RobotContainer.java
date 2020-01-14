package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.autonomous.LiveDashboard;
import frc.robot.commands.Drive;
import frc.robot.commands.Drive.State;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchEffector;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public static AHRS navX;
    private static NetworkTable limelight;
    public static LiveDashboard falcondash;
    public static Drivetrain drivetrain;
    public static Shooter shooter;
    public static HatchEffector hatcheffector;

    public static XboxController driver = new XboxController(0);
    public static JoystickButton driver_B = new JoystickButton(driver, 2);
    public static JoystickButton driver_Y = new JoystickButton(driver, 4);

    public static Joystick operator = new Joystick(1);
    public static JoystickButton trigger = new JoystickButton(operator, 1);
    public static JoystickButton operator_2 = new JoystickButton(operator, 2);
    public static JoystickButton operator_3 = new JoystickButton(operator, 3);

    /* Instantiates all robot components */
    public RobotContainer(){
        navX = new AHRS(Port.kMXP);

        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(State.CheesyDriveOpenLoop));

        shooter = Shooter.getInstance();

        hatcheffector = HatchEffector.getInstance();

        falcondash = new LiveDashboard();
        falcondash.setup();

        bindOI();
    }

    /* Binding OI input to Commands */
    private void bindOI() {
        if(shooter != null) {
            trigger.whileHeld( new RunCommand(()-> Shooter.setOpenLoop(-operator.getY(), operator.getY()))).whenReleased( ()-> Shooter.setOpenLoop(0.0, 0.0));
            operator_2.whenPressed( ()-> Shooter.setOpenLoop(0.65, -0.39)).whenReleased( ()-> Shooter.setOpenLoop(0.0, 0.0));
            operator_3.whenPressed( ()-> Shooter.setOpenLoop(-0.3, 0.3)).whenReleased( ()-> Shooter.setOpenLoop(0.0, 0.0));
        }

        if(hatcheffector != null) {
            // Releases retainer if down, and then ejects
            driver_B.whenPressed(new SequentialCommandGroup(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new InstantCommand(()->hatcheffector.release()),
                        new WaitCommand(0.125)
                    ),
                    new InstantCommand(),
                    ()->HatchEffector.retainer.get() == Value.kReverse
                ),
                new InstantCommand(()->hatcheffector.extend()),
                new WaitCommand(0.125),
                new InstantCommand(()->hatcheffector.retract())
            ));

            driver_Y.whenPressed(()-> hatcheffector.alternate_retainer());
        }
    }

    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. This method corrects that by returning the opposite of the value
        return -deadbandX(driver.getY(Hand.kLeft), DriverConstants.kJoystickDeadband);
    }

    public static double getTurnValue() {
        return deadbandX(driver.getX(Hand.kRight), DriverConstants.kJoystickDeadband);
    }

    public static double getGyroDegrees(){
        return navX.getFusedHeading();
    }

    public static double getGyroRadians(){
        double rad = navX.getFusedHeading() * Math.PI/180.0;
        rad = -rad; // Making the angle measurement CCW Positive (navX natively CW Positive)

        return rad;
    }
    
    public static double deadbandX(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    public static double exponentiate(double input, double power) {
        return Math.copySign(Math.abs(Math.pow(input, power)), input);
    }

    public static double deadbandY(double input, double deadband) {
        if (Math.abs(input) == 0.0) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return input * (1.0 - deadband) + Math.signum(input) * deadband;
        }
    }

     /*
     * Methods for setting limelight values
     */

    public void setLEDMode(LEDMode ledMode) {
        limelight.getEntry("ledMode").setNumber(ledMode.val);
    }

    public void setCamMode() {
        limelight.getEntry("camMode").setNumber(0);
    }

    public void setStreamMode(StreamMode stream) {
        limelight.getEntry("stream").setNumber(stream.val);
    }

    public void setPipeline(VisionPipeline pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline.val);
    }

    public static double getXOffset() {
        return -limelight.getEntry("tx").getDouble(0);
    }

    public static double getYOffset() {
        return -limelight.getEntry("ty").getDouble(0.0);
    }

    public static enum LEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        public int val;

        private LEDMode(int val) {
            this.val = val;
        }
    }

    public static enum StreamMode {
        SIDE_BY_SIDE(0), PIP_MAIN(1), PIP_SECONDARY(2);

        public int val;
        
        private StreamMode(int val){
            this.val = val;
        }
    }

    public static enum VisionPipeline {
        VISION(0), DRIVER(1);

        public int val;

        private VisionPipeline(int val){
            this.val = val;
        }
    }
}
