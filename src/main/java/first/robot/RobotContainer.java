// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;

import first.robot.Constants.OperatorConstants;
import first.robot.subsystems.DriveTrain;
import first.robot.subsystems.ExampleSubsystem;
import first.robot.subsystems.LimelightFront;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double period = 0;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final LimelightFront m_LimelightFront = new LimelightFront();
  private final DriveTrain drivetrain = new DriveTrain(m_LimelightFront);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGamepad m_driverController =
      new CommandGamepad(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(double period) {
    this.period = period;
    drivetrain.setDefaultCommand(
        drivetrain.driveFieldRel(
            () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(), this.period
          ));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
   * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.eastFace().onTrue(m_exampleSubsystem.exampleMethodCommand());

    //  // While holding R1, drive at half speed
    //  m_driverController
    //      .rightBumper()
    //      .onTrue(Command.noRequirements(_ -> robotDrive.setMaxOutput(0.5)).named("Set half speed"))
    //      .onFalse(Command.noRequirements(_ -> robotDrive.setMaxOutput(1)).named("Set full speed"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; //Autos.exampleAuto(m_exampleSubsystem);
  }
}
