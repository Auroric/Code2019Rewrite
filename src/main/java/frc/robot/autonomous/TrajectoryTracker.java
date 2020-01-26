package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryTracker extends RamseteCommand {

    static DifferentialDriveKinematics kinematics = Drivetrain.kinematics;
    static RamseteController follower = new RamseteController(2, 0.7);
    private double startTime;
    private Trajectory trajectory;

    public TrajectoryTracker(Trajectory trajectory) {
        super(trajectory, 
              () -> Drivetrain.odometry.getPoseMeters(), 
              follower, 
              kinematics,
              Drivetrain::setClosedLoop
        );

        this.trajectory = trajectory;
    }

    public TrajectoryTracker(Trajectory trajectory, boolean useWPILIB) {
        super(trajectory, 
              () -> Drivetrain.odometry.getPoseMeters(),
              follower,
              Drivetrain.feedforward,
              kinematics,
              Drivetrain::getWheelSpeeds,
              Drivetrain.leftPIDController,
              Drivetrain.rightPIDController,
              (left, right) -> Drivetrain.setOpenLoop(left, right, true),
              RobotContainer.drivetrain);
        this.trajectory = trajectory;
    }

    @Override 
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();

    }

    @Override
    public void execute() {
        super.execute();
        double currentTime = Timer.getFPGATimestamp();

        Trajectory.State currentState = trajectory.sample(currentTime-startTime);
        Pose2d currentPose = currentState.poseMeters;

        RobotContainer.falcondash.putPath(Drivetrain.meterToFeet(currentPose));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Drivetrain.stopMotors();
        RobotContainer.falcondash.endPath();
    }
}
