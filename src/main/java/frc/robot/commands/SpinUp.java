// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class SpinUp extends Command {

    // FIX: private final + m_ prefix per WPILib convention
    private final FuelSubsystem m_fuelSubsystem;

    // FIX: Yorum düzeltildi — "Intake" → "SpinUp"
    /** Creates a new SpinUp. */
    public SpinUp(FuelSubsystem fuelSubsystem) {
        m_fuelSubsystem = fuelSubsystem;
        addRequirements(fuelSubsystem);
    }

    // Spin up the launcher rollers to launch speed, hold feeder back
    @Override
    public void initialize() {
        m_fuelSubsystem.setIntakeLauncherRoller(
            SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
        // FIX: Key düzeltildi — FuelSubsystem'deki key ile eşleşiyordccf
    }

    // Launcher speed is set in initialize(), no updates needed
    @Override
    public void execute() {}

    // FIX: Stop only if interrupted — if finished normally, Launch command takes over
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_fuelSubsystem.stop();
        }
    }

    // Ends via withTimeout() in LaunchSequence
    @Override
    public boolean isFinished() {
        return false;
    }
}