// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Stability balls are the best :)

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnGyro extends CommandBase {
  // Defining our variables and drivetrain to be used later on
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_degrees;

  /**
   * Creates a new TurnGyro instance.
   *
   * @param speed The speed at which the robot will turn, 1 being fastest and 0 being completely stopped
   * (0.3ish is the slowest the robot will physically go). Negative values = reverse turning
   * @param degrees How many degrees to turn (using gyro)
   * @param drive The drive subsystem on which this command will run, required so that multiple methods
   * cannot be sending conflicting values to the motors
   */
  public TurnGyro(double speed, double degrees, Drivetrain drive) {
    // Telling the program that our variables should be what we passed into the method
    m_speed = speed;
    m_degrees = degrees;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stop motors, just to be safe
    m_drive.arcadeDrive(0, 0);
    // Reset gyro for starting point
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Telling the motors to continually move at 0 degrees x (forward) and
    // whatever defined speed to turn (this results in a turn-in-place)
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop driving once the turn is complete
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This is the logic that tells the command when it should end; the bulk of this method's smarts
    return Math.abs(m_drive.getGyroAngleZ()) >= m_degrees;
  }
}