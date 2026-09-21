// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.filter.MedianFilter;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.VisionConstants;
//import first.robot.utils.LimelightHelpers;
import com.limelightvision.RobotOrientationData;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveTrain extends Mechanism
{

  private final SwerveModule m_frontLeft =
    new SwerveModule(
      DriveConstants.FRONTLEFTDRIVEMOTORID,
      DriveConstants.FRONTLEFTTURNINGMOTORID,
      DriveConstants.FRONTLEFTTURNINGENCODERID);

  private final SwerveModule m_rearLeft =
    new SwerveModule(
      DriveConstants.REARLEFTDRIVEMOTORID,
      DriveConstants.REARLEFTTURNINGMOTORID,
      DriveConstants.REARLEFTTURNINGENCODERID);

  private final SwerveModule m_frontRight =
    new SwerveModule(
      DriveConstants.FRONTRIGHTDRIVEMOTORID,
      DriveConstants.FRONTRIGHTTURNINGMOTORID,
      DriveConstants.FRONTRIGHTTURNINGENCODERID);

  private final SwerveModule m_rearRight =
    new SwerveModule(
      DriveConstants.REARRIGHTDRIVEMOTORID,
      DriveConstants.REARRIGHTTURNINGMOTORID,
      DriveConstants.REARRIGHTTURNINGENCODERID);

  //private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final com.ctre.phoenix6.hardware.Pigeon2 gyro =
    new com.ctre.phoenix6.hardware.Pigeon2(1, DriveConstants.ctre_bus);

  public double driveYaw;
  public double driveYawDirection;
  public double driveYawOffset;
  public boolean fieldRelative = true;
  SlewRateLimiter xRateLimiter, yRateLimiter;
  boolean slowMode;

  // SwerveDriveOdometry odometry;
  public SwerveDrivePoseEstimator poseEstimator;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(.1));
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.35, 0.35, Units.degreesToRadians(999.));

  public double poseX, poseY, poseYaw;
  Field2d field;

  LimelightFront limelightFront;
  MedianFilter limelightXFilter;
  MedianFilter limelightYFilter;
  MedianFilter limelightYawFilter;
  double compositeLatency;
  Pose2d compositeVisionPose;
  boolean isVisionValid;
  double lastVisionUpdate;
  Integer[] hubTagArray = {2, 5, 8, 9 ,10, 11, 18, 21, 24, 25, 26, 27};
  List<Integer> hubTagIDs = Arrays.asList(hubTagArray);

  public DriveTrain(LimelightFront llf) {

    // odometry =
    //   new SwerveDriveOdometry(
    //       DriveConstants.DRIVEKINEMATICS,
    //       getRotation2d(),
    //       new SwerveModulePosition[] {
    //         m_frontLeft.getPosition(),
    //         m_frontRight.getPosition(),
    //         m_rearLeft.getPosition(),
    //         m_rearRight.getPosition()
    //       });

    Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
    gyroConfig.MountPose.MountPoseYaw = 180;
    gyroConfig.MountPose.MountPoseRoll = 180;
    gyro.getConfigurator().apply(gyroConfig);
    gyro.setYaw(0);

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVEKINEMATICS, 
      getRotation2d(), 
      getModulePositions(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );

    poseX = 0;
    poseY = 0;
    poseYaw = 0;
    field = new Field2d();

    driveYaw = 0;
    driveYawDirection = 0;
    driveYawOffset = 0;
    xRateLimiter = new SlewRateLimiter(1 / DriveConstants.RAMP_TIME);
    yRateLimiter = new SlewRateLimiter(1 / DriveConstants.RAMP_TIME);
    slowMode = false;

    limelightFront = llf;
    limelightXFilter = new MedianFilter(3);
    limelightYFilter = new MedianFilter(3);
    limelightYawFilter = new MedianFilter(3);
    compositeLatency = 0;
    compositeVisionPose = new Pose2d();
    lastVisionUpdate = 0;
    isVisionValid = true;
    this.setDefaultCommand(this.periodic());
  }

  public Command periodic()
  {
    return this.run( 
      coro -> {

        driveYaw = gyro.getYaw().getValueAsDouble() + driveYawOffset;
        driveYaw = MathUtil.angleModulus(Math.toRadians(driveYaw));
        driveYaw = Math.toDegrees(driveYaw);

        // update pose estimator
        poseEstimator.update(getRotation2d(), getModulePositions());

        limelightFront.setOrientation(getHeading());

        processFrame();

        if (RobotState.isDisabled()) {
          if (flipPath()) {
            driveYawOffset = 180;
          } else {
            driveYawOffset = 0;
          }
          fixPose();
        }

        // add Vision to pose estimator
        if (isVisionValid) {
          poseEstimator.addVisionMeasurement(
            compositeVisionPose,
            Timer.getTimestamp() - (compositeLatency / 1000),
            visionMeasurementStdDevs);
        }
      
        // update pose variables
        poseX = poseEstimator.getEstimatedPosition().getX();
        poseY = poseEstimator.getEstimatedPosition().getY();
        poseYaw = poseEstimator.getEstimatedPosition().getRotation().getDegrees();

        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("pose yaw", poseYaw);
        SmartDashboard.putNumber("x pose", poseX);
        SmartDashboard.putNumber("y pose", poseY);
        SmartDashboard.putNumber("FL cancoder", m_frontLeft.getPosition().angle.getRotations());
        SmartDashboard.putNumber("FR cancoder", m_frontRight.getPosition().angle.getRotations());
        SmartDashboard.putNumber("BL cancoder", m_rearLeft.getPosition().angle.getRotations());
        SmartDashboard.putNumber("BR cancoder", m_rearRight.getPosition().angle.getRotations());

      }
    ).named("periodic");
  }

  public boolean flipPath() {
        if (MatchState.getAlliance().isPresent()) {
          return MatchState.getAlliance().get() == Alliance.RED;
        }
        return false;
  }

  public void stop()
  {
    var chassisVelocities = new ChassisVelocities(0., 0., 0.);
    var velocities =
        SwerveDriveKinematics.desaturateWheelVelocities(
            DriveConstants.DRIVEKINEMATICS.toWheelVelocities(chassisVelocities),
            DriveConstants.MAXSPEEDMETERSPERSECOND);
    setModuleStates(velocities);
  }

   public Command driveRobotRel(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, double period) {
    return runRepeatedly(() -> this.drive(
      x.getAsDouble(),
      y.getAsDouble(),
      rot.getAsDouble(),
      false, period)
    ).named("SwerveDrive");
  }

   public Command driveFieldRel(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, double period) {
    return runRepeatedly(() -> this.drive(
      x.getAsDouble(),
      y.getAsDouble(),
      rot.getAsDouble(),
      true, period)
    ).named("SwerveDrive");
  }

  public void drive(
      double xVelocity, double yVelocity, double rot, boolean fieldRelative, double period) {
    var chassisVelocities = new ChassisVelocities(xVelocity, yVelocity, rot);
    if (fieldRelative) {
      chassisVelocities = chassisVelocities.toRobotRelative(getRotation2d());
    }
    chassisVelocities = chassisVelocities.discretize(period);

    var velocities =
        SwerveDriveKinematics.desaturateWheelVelocities(
            DriveConstants.DRIVEKINEMATICS.toWheelVelocities(chassisVelocities),
            DriveConstants.MAXSPEEDMETERSPERSECOND);

    m_frontLeft.setDesiredVelocity(velocities[0]);
    m_frontRight.setDesiredVelocity(velocities[1]);
    m_rearLeft.setDesiredVelocity(velocities[2]);
    m_rearRight.setDesiredVelocity(velocities[3]);
  }

  public void setModuleStates(SwerveModuleVelocity[] velocities)
  {
    m_frontLeft.setDesiredVelocity(velocities[0]);
    m_frontRight.setDesiredVelocity(velocities[1]);
    m_rearLeft.setDesiredVelocity(velocities[2]);
    m_rearRight.setDesiredVelocity(velocities[3]);
  }

  public SwerveModuleVelocity[] getModuleStates()
  {
    SwerveModuleVelocity[] states = {
        m_frontLeft.getVelocity(),
        m_frontRight.getVelocity(),
        m_rearLeft.getVelocity(),
        m_rearRight.getVelocity()
      };
    return states;
  }

  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] m_positions = {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      };
    return m_positions;
  }

  public void resetEncoders()
  {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() { gyro.setYaw(0.); gyro.reset();}

  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(
      gyro.getYaw().getValueAsDouble()
      //gyro.getRotation2d().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)
    );
  }

  public double getHeading()
  {
    return gyro.getYaw().getValueAsDouble(); //getRotation2d().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Pose2d getPose() { return poseEstimator.getEstimatedPosition(); };

  public void fixPose() {
    poseEstimator.resetPose(new Pose2d(compositeVisionPose.getTranslation(), Rotation2d.fromDegrees(getHeading())));
  }

  public void processFrame() {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isVisionValid = false;

    if (limelightFront.acceptPose()) {
      double targetArea = limelightFront.getTargetArea();
      if (targetArea > VisionConstants.TARGET_AREA_THRESHHOLD) {
        Pose2d pose = limelightFront.getBotPose2dMT2();
        totalArea += targetArea;
        x += pose.getX() * targetArea;
        y += pose.getY() * targetArea;
        yaw += pose.getRotation().getDegrees() * targetArea;
        compositeLatency += limelightFront.getLatency();
      }
    } 

    SmartDashboard.putNumber("Total tag area", totalArea);

    if (totalArea < VisionConstants.TOTAL_TARGET_AREA_THRESHHOLD) {
      isVisionValid = false;
      compositeLatency = 0;
    } else {
      isVisionValid = true;
      x /= totalArea;
      y /= totalArea;
      yaw /= totalArea;
      compositeLatency /= totalArea;

      compositeVisionPose = new Pose2d(
        limelightXFilter.calculate(x),
        limelightYFilter.calculate(y),
        Rotation2d.fromDegrees(limelightYawFilter.calculate(yaw))
      );
    }

  }

}