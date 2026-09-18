// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.ModuleConstants;;

class TurningEncoder {
  private final CANcoder m_cancoder;

  public TurningEncoder (int ID) {
    m_cancoder = new CANcoder(ID, DriveConstants.ctre_bus);
  }

  public void reset() {
    m_cancoder.setPosition(0.);
  }

  public double getPositionRadians() {
    return m_cancoder.getAbsolutePosition().getValueAsDouble() * (2.*Math.PI);
  }
}

class DriveEncoder {
  private final RelativeEncoder m_encoder;
  private double m_unitsPerRotation;

  public DriveEncoder (SparkMax driveMotor, double unitsPerRotation) {
    m_encoder = driveMotor.getEncoder();
    m_unitsPerRotation = unitsPerRotation;
  }

  public void reset() {
    m_encoder.setPosition(0.);
  }

  public double getPosition() {
    return m_encoder.getPosition().get() * m_unitsPerRotation;
  }

  public double getVelocity() {
    return m_encoder.getVelocity().get() * m_unitsPerRotation / 60;
  }
}

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final TurningEncoder m_turningEncoder;
  private final DriveEncoder m_driveEncoder;

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.PMODULETURNINGCONTROLLER,
          ModuleConstants.IMODULETURNINGCONTROLLER,
          ModuleConstants.DMODULETURNINGCONTROLLER,
          new TrapezoidProfile.Constraints(
              ModuleConstants.MAXMODULEANGULARSPEEDRADIANSPERSECOND,
              ModuleConstants.MAXMODULEANGULARACCELERATIONRADIANSPERSECONDSQUARED));

  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID) {
        

    m_driveMotor = new SparkMax(CANBusMap.CAN_S0, driveMotorID,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new SparkMax(CANBusMap.CAN_S0, turningMotorID,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

    driveMotorConfig.smartCurrentLimit(40);
    driveMotorConfig.closedLoopRampRate(.8);
    driveMotorConfig.signals.primaryEncoderPositionPeriodMs(100);
    driveMotorConfig.signals.primaryEncoderPositionPeriodMs(20);
    driveMotorConfig.signals.primaryEncoderPositionPeriodMs(20);
    driveMotorConfig.idleMode(IdleMode.kBrake);
    m_driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    turningMotorConfig.smartCurrentLimit(25);
    turningMotorConfig.closedLoopRampRate(.8);
    turningMotorConfig.signals.primaryEncoderPositionPeriodMs(100);
    turningMotorConfig.signals.primaryEncoderPositionPeriodMs(20);
    turningMotorConfig.signals.primaryEncoderPositionPeriodMs(20);
    turningMotorConfig.idleMode(IdleMode.kBrake);
    m_turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_driveEncoder = new DriveEncoder(m_driveMotor,
                           ModuleConstants.DRIVEENCODERDISTANCEPERROTATION);

    m_turningEncoder = new TurningEncoder(turningEncoderID);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(),
                 new Rotation2d(m_turningEncoder.getPositionRadians()));
  }

  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPositionRadians()));
  }

  /**
   * Sets the desired velocity for the module.
   *
   * @param desiredVelocity Desired velocity.
   */
  public void setDesiredVelocity(SwerveModuleVelocity desiredVelocity) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPositionRadians());

    // Optimize the desired velocity to avoid spinning further than 90 degrees, then scale velocity
    // by cosine of angle error. This scales down movement perpendicular to the desired direction of
    // travel that can occur when modules change directions. This results in smoother driving.
    SwerveModuleVelocity velocity =
        desiredVelocity.optimize(encoderRotation).cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller and feedforward.

    // final double driveOutput =
    //     m_drivePIDController.calculate(driveEncoder.getRate(), velocity.velocity)
    //         + driveFeedforward.calculate(desiredVelocity.velocity);

    final double driveOutput =desiredVelocity.velocity;

    // Calculate the turning motor output from the turning PID controller and feedforward.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPositionRadians(), velocity.angle.getRadians());

    m_driveMotor.setThrottle(driveOutput);
    m_turningMotor.setThrottle(turnOutput);
  }

  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }
}