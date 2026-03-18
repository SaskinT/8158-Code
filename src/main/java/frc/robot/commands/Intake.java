// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class Intake extends Command {

    // FIX: private final + m_ prefix per WPILib convention
    private final FuelSubsystem m_fuelSubsystem;

    public Intake(FuelSubsystem fuelSubsystem) {
        m_fuelSubsystem = fuelSubsystem;
        addRequirements(fuelSubsystem);
    }

    // Set the rollers to the appropriate values for intaking
    @Override
    public void initialize() {
        m_fuelSubsystem.setIntakeLauncherRoller(
            SmartDashboard.getNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT));
        m_fuelSubsystem.setFeederRoller(
            SmartDashboard.getNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT));
    }

    // This command doesn't require updating any values while running
    @Override
    public void execute() {}

    // FIX: end() now delegates to stop() — no code duplication
    @Override
    public void end(boolean interrupted) {
        m_fuelSubsystem.stop();
    }

    // Command runs until interrupted (button released)
    @Override
    public boolean isFinished() {
        return false;
    }
}