// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class Launch extends Command {

    // FIX: private final + m_ prefix per WPILib convention
    private final FuelSubsystem m_fuelSubsystem;

    // FIX: Yorum düzeltildi — "Intake" → "Launch"
    /** Creates a new Launch. */
    public Launch(FuelSubsystem fuelSubsystem) {
        m_fuelSubsystem = fuelSubsystem;
        addRequirements(fuelSubsystem);
    }

    // Set the rollers to launch speed and feed the fuel
    @Override
    public void initialize() {
        m_fuelSubsystem.setIntakeLauncherRoller(
            SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
        m_fuelSubsystem.setFeederRoller(
            SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
    }

    // Launcher speed is set in initialize(), no updates needed
    @Override
    public void execute() {}

    // FIX: Launch is the last command in LaunchSequence — always stop on end
    @Override
    public void end(boolean interrupted) {
        m_fuelSubsystem.stop();
    }

    // Runs until interrupted (button released)
    @Override
    public boolean isFinished() {
        return false;
    }
}