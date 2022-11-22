// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArmCommand extends CommandBase {
    // Define the supplier vars and subsystem var
    private final Supplier<Double> m_liftSupplier;
    private final Supplier<Double> m_tiltSupplier;
    private final Supplier<Double> m_gripSupplier;
    private final ArmSubsystem m_armSubsystem;

    /**
     * Creates a new ArmCommand. This command has the ability to drive the Romi's arm extention using the
     * ArmSubsystem.
     * @param armSubsystem The arm subsystem that this command will run on
     * @param liftSupplier Lift supplier (Lambda); 0-1
     * @param tiltSupplier Tilt supplier (Lambda); 0-1
     * @param gripSupplier Grip supplier (Lambda); 0-1
     */
    public ArmCommand(
        // Define the arguments for the ArmCommand; the suppliers and subsystem
        Supplier<Double> liftSupplier,
        Supplier<Double> tiltSupplier,
        Supplier<Double> gripSupplier,
        ArmSubsystem armSubsystem) {

        // Basically telling the program that the arguments are the same thing as the
        // supplier and subsystem vars we defined earlier on
        m_liftSupplier = liftSupplier;
        m_tiltSupplier = tiltSupplier;
        m_gripSupplier = gripSupplier;
        m_armSubsystem = armSubsystem;

        // Reqire the arm subsystem in the command so that we can manipulate the arm
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Pass the suppliers into the actual subsystem (servos)
        m_armSubsystem.m_lift.set(m_liftSupplier.get());
        m_armSubsystem.m_tilt.set(m_tiltSupplier.get());
        m_armSubsystem.m_grip.set(m_gripSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // This command shouldn't end because then the arm would just...cease to work
        // If we want to move it back to initial position, we can just zero values
        return false;
    }
}
