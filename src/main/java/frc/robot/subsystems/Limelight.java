package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.AprilTagConstants;

public class Limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.limelightName);
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    NetworkTableEntry tid;
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
    public double yOffset = LimelightConstants.limelightYOffsetMeters;
    public double angleChosen = 40.0;
    public int trackingID = 0;
    public boolean enabled = LimelightConstants.limelightEnabled;
    int currentPipeline = -1;
    boolean isNetworkTableConnected;

    Translation2d offset;

    private static Limelight mainLimelight;

    public static InterpolatingDoubleTreeMap angleChooser = new InterpolatingDoubleTreeMap();
    static {
      // Put any yOffset values here with the corresponding angle.
      angleChooser.put(1.25, 0.0);
    }

    public static Limelight getLimelightInstance() {
        if (mainLimelight == null) {
          mainLimelight = new Limelight(LimelightConstants.limelightEnabled);
        }
        return mainLimelight;
    }

    public Limelight(boolean enabled) {
        this.enabled = enabled;
    }

    /*
     * Methods Like This Will Be Used When A Certain AprilTag Always Has The Same Angle When Using It's Function.
     * The Source Only Has One Angle, So It Fits Within This Description.
     * More Methods Like This May Be Added Later
     */
    public void sourceAprilTag() {
      angleChosen = 40;
    }
    public void ampAprilTag(){

    }
    public void speakerAprilTag(){
      angleChosen = angleChooser.get(yOffset);
    }

    @Override
    public void periodic() {
      if (getTargetVisible()){
        // Set yOffset
        ty = table.getEntry("ty");
        yOffset = ty.getDouble(0.0);
        // Set tracking ID (This will be used later so we know which tags are important)
        tid = table.getEntry("tid");
        trackingID = (int) tid.getDouble(0.0);

        // Add AprilTag ID Logic Here.
        /*if(DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))){
          switch (trackingID) {
            case AprilTagConstants.sourceBlueLeft:
              sourceAprilTag();
              break;
            case AprilTagConstants.sourceBlueRight:
              sourceAprilTag();
              break;
            case AprilTagConstants.ampBlue:
              ampAprilTag();
              break;
            case AprilTagConstants.speakerBlueLeft:
              speakerAprilTag();
              break;
            case AprilTagConstants.speakerBlueRight:
              speakerAprilTag();
              break;
            default:
              // Do nothing, the AprilTag is red.
              break;
          }
        } else if(DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))){
          switch (trackingID) {
            case AprilTagConstants.sourceRedLeft:
              sourceAprilTag();
              break;
            case AprilTagConstants.sourceRedRight:
              sourceAprilTag();
              break;
            case AprilTagConstants.ampRed:
              ampAprilTag();
              break;
            case AprilTagConstants.speakerRedLeft:
              speakerAprilTag();
              break;
            case AprilTagConstants.speakerRedRight:
              speakerAprilTag();
              break;
            default:
              // Do nothing, the AprilTag is blue.
              break;
          }
        } */

        angleChosen = yOffset; // This will be commented out.
      }
    }

    public boolean getTargetVisible() {
      if (enabled && isNetworkTableConnected) {
        if(tv.getDouble(0.0) == 1.0){
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    }
}

