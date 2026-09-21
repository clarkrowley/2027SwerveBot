// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import first.robot.Constants.VisionConstants;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import com.limelightvision.LEDMode;

public class Camera extends Mechanism {

  private final NetworkTable table;
  private final NetworkTableEntry latency;
  private final NetworkTableEntry tagId;
  private int currentTagId = 0;
  private AprilTagFieldLayout aprilTagLayout;
  private double fieldWidth;
  private double fieldLength;
  private final Limelight limelight;

  // IP IS - 10.19.12.11:5801 //

  /** Creates a new LimelightShooter. */
  public Camera() {
    limelight = new Limelight(getName());

    limelight.setPipelineIndex(0);
    table = NetworkTableInstance.getDefault().getTable(getName());
    latency = table.getEntry("tl");
    tagId = table.getEntry("tid");
    aprilTagLayout = AprilTagFieldLayout.loadField(VisionConstants.APRILTAG_FIELD);
    fieldWidth = aprilTagLayout.getFieldWidth();
    fieldLength = aprilTagLayout.getFieldLength();
    this.setDefaultCommand(this.periodic());
  }

  public Command periodic() {
    return this.run(
      coro -> {
        currentTagId = getTagId();
        SmartDashboard.putNumber(getPosition()+"Latency: ",(double) latency.getNumber(0));
        SmartDashboard.putNumber(getPosition()+"TagID: ",tagId.getInteger(-1));
        SmartDashboard.putBoolean(getPosition()+"HasBotPose: ",(currentTagId>0));
      }
    ).named("periodic");
  }

  /**
   * Gets the name of the limelight.
   * @return The name of the limelight as a string
   */
  public String getName() {
    return "limelight-null";
  }

  /**
   * set the robot orientation for MT2
   */
  public void setOrientation(double yawDegrees) {
    limelight.setRobotOrientation(yawDegrees, true);
  }

  /**
   * Gets the position of the limelight.
   * @return The position of the limelight as a string
   */
  public String getPosition() {
    return "null";
  }

  /**
   * Toggles between pipeline 0 and 1.
   */
  public void setPipeline(int pipeline) {
    limelight.setPipelineIndex(pipeline);
  }

  /**
   * Returns the Apriltags estimated pose or null if there is no pose.
   * @return The limelight's outputted pose
   */
  public Pose2d getBotPose2dMT1() {
    return limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE).pose;
  }

  public PoseEstimate getMT1PoseEstimate() {
    return limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
  }

  public Pose2d getBotPose2dMT2() {
    return limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE).pose;
  }

  public PoseEstimate getMT2PoseEstimate() {
    return limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);
  }

  /**
   * Returns the latency of the pipeline
   * @return
   */
  public double getLatency() {
    return latency.getDouble(0);
  }

  /**
   * Gets the ID of the biggest tag in frame.
   * @return The ID of the tag
   */
  public int getTagId() {
    return (int) tagId.getInteger(0);
  }

  /**
   * Gets the number of tags that the limelight can see.
   * @return The number of tags
   */
  public int getTagCount() {
    return limelight.getTargetCount();
  }

  /**
   * checks if the estimated bot pose is valid within the field.
   * @return boolean - field pose is valid
   */
  public boolean acceptPose() {
    if (limelight.getTargetCount() <= 0) {
      return false;
    }

    Pose2d pose = getBotPose2dMT2();
    double poseY = pose.getY();
    double poseX = pose.getX();

    if (poseY < 0) {
      return false;
    }

    if (poseX < 0) {
      return false;
    }

    if (poseY > fieldWidth) {
      return false;
    }

    if (poseX > fieldLength) {
      return false;
    }

    return true;
  }

  /**
   * Gets the X Offset of the tag from the center of the frame.
   * @return The X Offset
   */
  public double getXOffset() {
    return limelight.getTXDegrees();
  }

  /**
   * Gets the Y Offset of the tag from the center of frame.
   * @return The Y Offset
   */
  public double getYOffset() {
    return limelight.getTYDegrees();
  }

  /**
   * Gets the area of the tag in frame
   * @return The area of the tag
   */
  public double getTargetArea() {
    return limelight.getTargetAreaPercent();
  }

  /**
   * Turns the limelight LEDs on
   */
  public void setLedsOn() {
    limelight.setLEDMode(LEDMode.FORCE_ON);
  }

  // BFR - this appears to be unused
  public int getPipeline() {
    return limelight.getCurrentPipelineIndex();
  }
}
