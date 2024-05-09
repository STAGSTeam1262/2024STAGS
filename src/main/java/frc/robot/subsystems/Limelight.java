package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.limelightName);
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    public double distanceFromLimelightToGoalInches = 0.0;
    public double angleToGoalDegrees;
    NetworkTableEntry ledMode;
    NetworkTableEntry camMode;
    NetworkTableEntry pipeline;

    ShuffleboardTab tab;
    GenericEntry distanceToTargetX;
    GenericEntry distanceToTargetY;
    GenericEntry targetVisible;

    String name;
    double limeHeight = LimelightConstants.limelightHeight;
    double limeAngle = LimelightConstants.limelightAngle;
    double speakerHeight = 80.43;
    double yOffsetMeters = LimelightConstants.limelightYOffsetMeters;
    public boolean enabled = LimelightConstants.limelightEnabled;
    int currentPipeline = -1;
    boolean isNetworkTableConnected;

    Translation2d offset;

    private static Limelight mainLimelight;

    public static Limelight getLimelightInstance() {
        if (mainLimelight == null) {
          mainLimelight = new Limelight(LimelightConstants.limelightEnabled);
        }
        return mainLimelight;
    }

    public Limelight(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
      if (getTargetVisible()){
        // Set yOffset
        ty = table.getEntry("ty");
        yOffsetMeters = ty.getDouble(0.0);

    // Get Angle
        angleToGoalDegrees = limeAngle + yOffsetMeters;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); // Backup
      }
    }

  public boolean getTargetVisible() {
      if (enabled && isNetworkTableConnected) {
        return tv.getDouble(0.0) == 1.0;
      } else {
        return false;
      }
    }
}

