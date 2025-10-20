package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.*;
import frc.robot.util.LimelightHelpers;
import java.util.Arrays;
import java.util.List;

public class Vision extends SubsystemBase {
  private NetworkTable leftTable;
  private NetworkTable rightTable;
  final List<Integer> blueTagIds = Arrays.asList(12, 13, 16, 17, 18, 19, 20, 21, 22);
  final List<Integer> redTagIds = Arrays.asList(1, 2, 3, 6, 7, 8, 9, 10, 11);
  final List<Integer> tagIds;

  public Vision() {
    // Example camera initialization - you would add your actual cameras here
    // addCamera("front", "limelight-front", LimelightVersion.V4);
    // addCamera("back", "limelight-back", LimelightVersion.V3);

    leftTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_TABLE);
    rightTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_TABLE);

    tagIds =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redTagIds : blueTagIds;
  }

  @Override
  public void periodic() {
    if (leftTable == null) {
      leftTable = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_TABLE);
    }

    if (rightTable == null) {
      rightTable =
          NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_TABLE);
    }
  }

  public NetworkTable getLeftTable() {
    return leftTable;
  }

  public NetworkTable getRightTable() {
    return rightTable;
  }

  public double getArea(NetworkTable table) {
    return table.getEntry("ta").getDouble(0.0);
  }

  public double getXOffset(NetworkTable table) {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getYOffset(NetworkTable table) {
    return table.getEntry("ty").getDouble(0.0);
  }

  public int getTagId(NetworkTable table) {
    return (int) table.getEntry("tid").getInteger(0);
  }

  public String getBestLimelight() {
    final NetworkTable leftLimelight = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LEFT_TABLE);
    final NetworkTable rightLimelight = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.RIGHT_TABLE);

    LimelightHelpers.PoseEstimate leftPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    LimelightHelpers.PoseEstimate rightPerimeterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

    String bestLimelight = "";

    if (leftPerimeterMeasurement != null && tagIds.contains(getTagId(leftLimelight))) {
      bestLimelight = "limelight-left";
    }

    if (rightPerimeterMeasurement != null
        && tagIds.contains(getTagId(rightLimelight))
        && ((leftPerimeterMeasurement != null
                && rightPerimeterMeasurement.avgTagArea > leftPerimeterMeasurement.avgTagArea)
            || bestLimelight.isEmpty())) {
      bestLimelight = "limelight-right";
    }

    return bestLimelight;
  }
}
