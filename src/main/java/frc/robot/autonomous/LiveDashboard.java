package frc.robot.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;

// Interfaces with FalconDashboard 
// https://github.com/5190GreenHopeRobotics/FalconDashboard

public class LiveDashboard {
    private NetworkTableEntry robotX;
    private NetworkTableEntry robotY;
    private NetworkTableEntry robotHeading;

    private NetworkTableEntry followingPath;
    private NetworkTableEntry pathX;
    private NetworkTableEntry pathY;
    private NetworkTableEntry pathHeading;

    public void setup(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
        robotX = table.getEntry("robotX");
        robotY = table.getEntry("robotY");
        robotHeading = table.getEntry("robotHeading");

        followingPath = table.getEntry("isFollowingPath");
        pathX = table.getEntry("pathX");
        pathY = table.getEntry("pathY");
        pathHeading = table.getEntry("pathHeading");

        endPath();
        
    }

    public void putOdom(Pose2d pose){
        this.robotX.setDouble(pose.getTranslation().getX());
        this.robotY.setDouble(pose.getTranslation().getY());
        this.robotHeading.setDouble(pose.getRotation().getRadians());
    }

    public void putPath(Pose2d pose){
        this.pathX.setDouble(pose.getTranslation().getX());
        this.pathY.setDouble(pose.getTranslation().getY());
        this.pathHeading.setDouble(pose.getRotation().getRadians());
        this.followingPath.setBoolean(true);
    }

    public void endPath(){
        this.followingPath.setBoolean(false);
    }

}