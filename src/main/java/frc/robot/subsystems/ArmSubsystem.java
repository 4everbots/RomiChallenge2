// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Reference external I/O table:
 * 
 *       DIO      PWM      AnalogIn
 * EXT0    8        2           N/A
 * EXT1    9        3             0
 * EXT2   10        4             1
 * EXT4   11        5             2
 * EXT5   12        6             3
 */

/**
 * This class is used to control operations of the Romi's Robot Arm addon.
 * <p>It controls all three servos of the motor; lift, tilt, and gripper servos.
 * <p><i>Lift: EXT0 (PWM2), EXT1 (ANLG0)
 * <p>Tilt: EXT2 (PWM4), EXT4 (ANLG2)
 * <p>Gripper: EXT5 (PWM6)
 */
public class ArmSubsystem extends SubsystemBase{
    // Add comment
    public final Servo m_lift = new Servo(2);
    private final AnalogInput m_liftFeedback = new AnalogInput(0);
    public final Servo m_tilt = new Servo(4);
    private final AnalogInput m_tiltFeedback = new AnalogInput(2);
    public final Servo m_grip = new Servo(6);

    // Add comment
    public enum ChannelMode{
        SRVO,
        ANLG
    }

    // Add comment
    /**
     * RobotArm constructor:
     * @param SRVO
     * @param ANLG
     */
    public ArmSubsystem(ChannelMode SRVO, ChannelMode ANLG) {
        
    }

    public AnalogInput getLiftFeedback() {
        return m_liftFeedback;
    }

    public AnalogInput getTiltFeedback() {
        return m_tiltFeedback;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
