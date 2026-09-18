// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class ExampleSubsystem extends Mechanism {

  public ExampleSubsystem() {}

  public Command exampleMethodCommand() {
    return this.run(
        coro -> { }
      ).named("Example Command");

  }

}