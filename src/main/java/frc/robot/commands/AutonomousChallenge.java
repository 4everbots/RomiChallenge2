// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChallenge extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drive subsystem on which this command will run, required so that multiple methods
   * cannot be sending conflicting values to the motors
   */
  public AutonomousChallenge(Drivetrain drivetrain) {
    System.out.println("Stability balls rule!!");
    addCommands(
        new DriveDistanceCorrected(1, 251.81, drivetrain),
        new TurnDegrees(1, 87, drivetrain),
        new DriveDistanceCorrected(1, 69, drivetrain),
        new TurnDegrees(1, 87, drivetrain),
        new DriveDistanceCorrected(1, 269, drivetrain));


        // old commands
        // new DriveDistance(-0.5, 10, drivetrain),
        // new TurnDegrees(-0.5, 180, drivetrain),
        // new DriveDistance(-0.5, 10, drivetrain),
        // new TurnDegrees(0.5, 180, drivetrain));
  }
}
