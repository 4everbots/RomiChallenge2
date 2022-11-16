// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistanceCorrected extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_distance;

  // NOTES FOR TUNING PID!!
  // if we have implemented kI and kD properly, we should be able to decrease kP (eventually)
  // remember -- kP is looking at the present, kI at the past, and kD at the future
  //   - if the Romi is not moving enough when it's close to being straight, increase kI
  //   - if it is moving too much when it's close to being straight, decrease kI
  //   - if there isn't enough change from when we just had kP, then increase kI_limit
  //   - hence, if there is too much change, decrease kI_limit
  // for kD:
  //   - if the robot is randomly oscilating for no apparant reason, kD is likely too high
  //   - I'm not entirely sure the flags of if kD is too low. it's all too similar!! :(
  // possible useful links:
  //   - https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
  //   - https://www.youtube.com/watch?v=Z24fSBVJeGs
  // good luck!

  // PID constants
  private final double kP = 0.0885;
  private final double kI = 0.2;
  // kI related vars, inital setting
  private double errorSum = 0;
  private double lastTimestamp = 0;
  private final double kD = 0.05;
  // kD lastError initial
  private double lastError = 0;

  // Set up a limiting factor for kI; this will have it activate only within a 5 degree error
  private final double kI_limit = 5;

  /**
   * Creates a new DriveDistanceCorrected. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will turn, 1 being fastest and 0 being completely stopped
   * (0.3ish is the slowest the robot will physically go). Negative values = reverse turning
   * @param inches The number of inches the robot will drive
   * @param drive The drive subsystem on which this command will run, required so that multiple methods
   * cannot be sending conflicting values to the motors
   */
  public DriveDistanceCorrected(double speed, double inches, Drivetrain drive) {
    m_speed = speed;
    m_distance = inches;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stop motors, just to be safe
    m_drive.arcadeDrive(0, 0);
    // Reset gyro so we have a point of reference for keeping robot straight
    // This means that however the robot is facing when this method is called,
    // it will attempt to keep that angle (as it thinks that is "straight"; 0)
    m_drive.resetGyro();
    // Reset encoders so we have a point of reference for distance
    m_drive.resetEncoders();
    // Reset kI and kD-related vars
    errorSum = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = 0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the opposite of the Z gyro (this is our turning/drift error)
    double driveError = -m_drive.getGyroAngleZ();
    // Calculate deltaT and errorSum (both extentions of kI)
    double dT = Timer.getFPGATimestamp() - lastTimestamp;
    // Only calculate the errorSum (and therefore kI) if within the threshold of kI_limit
    if (Math.abs(driveError) < kI_limit){
      errorSum += driveError * dT;
    }
    //kD math
    double errorRate = (driveError - lastError) / dT;

    // PID math to pass into arcadeDrive turn
    double turnPower = kP * driveError + kI * errorSum + kD * errorRate;
    m_drive.arcadeDrive(m_speed, turnPower, false);

    // Update lastTimestamp and lastError for correct kI and kD values
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = driveError;

    // For testing and debugging
    SmartDashboard.putNumber("Error (kP term)", driveError);
    SmartDashboard.putNumber("Error Sum (kI term)", errorSum);
    SmartDashboard.putNumber("Error Rate (kD term)", errorRate);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop driving once the drive is complete
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}