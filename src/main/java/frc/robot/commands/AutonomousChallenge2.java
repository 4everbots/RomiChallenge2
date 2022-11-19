// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChallenge2 extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drive subsystem on which this command will run, required so that multiple methods
   * cannot be sending conflicting values to the motors
   */
  public AutonomousChallenge2(Drivetrain drivetrain) {
    System.out.println("4everbots™");
    addCommands(
      // We need to use DriveDistancePID as our driving method due to the Romi's tendancy to drift, but
      // we can just use plain old TurnDegrees (making use of the encoders) to turn because that isn't affected by drift

      // These values are just for current testing, we will replace them later with challenge values
      // (Yes, I am too lazy to set up the testing mode in the dashboard L)
      new DriveDistancePID(0.5, 50, drivetrain),
      new TurnDegrees(0.5, 90, drivetrain));
  }
}
