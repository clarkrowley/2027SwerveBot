// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.vision.apriltag.AprilTagFields;

import com.ctre.phoenix6.CANBus;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class VisionConstants {
    public static final double TARGET_AREA_THRESHHOLD = 0.05;
    public static final double TOTAL_TARGET_AREA_THRESHHOLD = 0.1;
    public static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltWelded;
  }

  public static class DriveConstants {

    public static CANBus ctre_bus = new CANBus("can_s0");
    public static final int FRONTLEFTDRIVEMOTORID = 1;
    public static final int FRONTLEFTTURNINGMOTORID = 2;
    public static final int FRONTLEFTTURNINGENCODERID = 1;

    public static final int FRONTRIGHTDRIVEMOTORID = 3;
    public static final int FRONTRIGHTTURNINGMOTORID = 4;
    public static final int FRONTRIGHTTURNINGENCODERID = 2;

    public static final int REARLEFTDRIVEMOTORID = 7;
    public static final int REARLEFTTURNINGMOTORID = 8;
    public static final int REARLEFTTURNINGENCODERID = 4;

    public static final int REARRIGHTDRIVEMOTORID = 5;
    public static final int REARRIGHTTURNINGMOTORID = 6;
    public static final int REARRIGHTTURNINGENCODERID = 3;

    public static final double TRACKWIDTH = Units.inchesToMeters(17.5);
    public static final double WHEELBASE = Units.inchesToMeters(17.5);
    public static final SwerveDriveKinematics DRIVEKINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2),    //FL
            new Translation2d(WHEELBASE / 2, -TRACKWIDTH / 2),   //FR
            new Translation2d(-WHEELBASE / 2, TRACKWIDTH / 2),   //RL
            new Translation2d(-WHEELBASE / 2, -TRACKWIDTH / 2)); //RR

    public static final double RAMP_TIME = 0.05;

    public static final boolean GYROREVERSED = false;
    public static final double OFFSETXTOROT = 0.05;

    public static final double SVOLTS = 1;
    public static final double VVOLTSECONDSPERMETER = 0.8;
    public static final double AVOLTSECONDSSQUAREDPERMETER = 0.15;

    public static final double MAXSPEEDMETERSPERSECOND = 4;
    public static final double MAXANGULARSPEEDRADIANSPERSECOND = 2*Math.PI;
    public static final double MAXANGULARSPEEDRADIANSPERSECONDSQUARED = Math.PI;
  }

  public static final class ModuleConstants {
    public static final double MAXMODULEANGULARSPEEDRADIANSPERSECOND = 10000.; //2 * 2 * Math.PI;
    public static final double MAXMODULEANGULARACCELERATIONRADIANSPERSECONDSQUARED = 10000.; //2 * 2 * Math.PI;

    public static final double WHEELDIAMETERMETERS = Units.inchesToMeters(4.0);
    public static final double WHEELGEARRATIO = 8.16;
    public static final double DRIVEENCODERDISTANCEPERROTATION =
      (WHEELDIAMETERMETERS * Math.PI) / WHEELGEARRATIO;

    public static final double PMODULETURNINGCONTROLLER = 0.25;
    public static final double IMODULETURNINGCONTROLLER = 0.0;
    public static final double DMODULETURNINGCONTROLLER = 0.0;

    //public static final double kPModuleDriveController = .7;
    /*
    public static final double kPTurningPid = 1.0;
    public static final double kITurningPid = 0.0;
    public static final double kDTurningPid = 0.1;
    */

  }

  public static class DriverController {
    public static final int DRIVERJOYSTICK = 0;
    //Controller Axes
    public static final int LEFT_Y_AXIS = 1; 
    public static final int LEFT_X_AXIS = 0; 
    public static final int RIGHT_Y_AXIS = 5; 
    public static final int RIGHT_X_AXIS = 4; 
    public static final int LEFT_TRIGGER = 2; 
    public static final int RIGHT_TRIGGER = 3;
    //Controller Buttons
    public static final int A_BUTTON = 1; 
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LT_BUMPER = 5;
    public static final int RT_BUMPER = 6; 
    public static final int BACK_BUTTON = 7;
  }

  public static class OperatorController {
    public static final int OPERATORJOYSTICK = 1;
    //Controller Axes
    public static final int LEFT_Y_AXIS = 1; 
    public static final int LEFT_X_AXIS = 0; 
    public static final int RIGHT_Y_AXIS = 5; 
    public static final int RIGHT_X_AXIS = 4; 
    public static final int LEFT_TRIGGER = 2; 
    public static final int RIGHT_TRIGGER = 3;
    //Controller Buttons
    public static final int A_BUTTON = 1; 
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LT_BUMPER = 5;
    public static final int RT_BUMPER = 6; 
    public static final int BACK_BUTTON = 7;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DeviceIDs {
    public static final int INTAKE_ARMS = 0;
    public static final int INTAKE_WHEELS = 1;

    public static final int INDEXER = 2;
    public static final int LEFT_SHOOTER = 3;
    public static final int RIGHT_SHOOTER = 4;
  }
}