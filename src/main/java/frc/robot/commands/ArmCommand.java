// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArmCommand extends CommandBase {
    // Add comment
    private final ArmSubsystem m_armSubsystem;
    private final Supplier<Double> m_liftSupplier;
    private final Supplier<Double> m_tiltSupplier;
    private final Supplier<Double> m_gripSupplier;

    /**
     * Creates a new ArmCommand. This command has the ability to drive the Romi's arm extention using the
     * ArmSubsystem.
     * @param armSubsystem Add comment
     * @param liftSupplier Add comment
     * @param tiltSupplier Add comment
     * @param gripSupplier Add comment
     */
    public ArmCommand(
        // Add comment
        ArmSubsystem armSubsystem,
        Supplier<Double> liftSupplier,
        Supplier<Double> tiltSupplier,
        Supplier<Double> gripSupplier) {

        // Add comment
        m_armSubsystem = armSubsystem;
        m_liftSupplier = liftSupplier;
        m_tiltSupplier = tiltSupplier;
        m_gripSupplier = gripSupplier;

        // Add comment
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
        return false;
    }
}
