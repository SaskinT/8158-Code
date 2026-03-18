// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {

    // FIX: Renamed to camelCase with m_ prefix per WPILib convention
    private final SparkMax m_leftIntakeLauncher;
    private final SparkMax m_rightIntakeLauncher;
    private final SparkMax m_indexer;

    public FuelSubsystem() {
        m_leftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        m_rightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        m_indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

        // FIX: Indexer (feeder) config created and properly applied
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
        indexerConfig.idleMode(IdleMode.kBrake);
        m_indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // FIX: Left launcher config — inverted=true, coast mode
        SparkMaxConfig leftLauncherConfig = new SparkMaxConfig();
        leftLauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        leftLauncherConfig.voltageCompensation(12);
        leftLauncherConfig.idleMode(IdleMode.kCoast);
        leftLauncherConfig.inverted(true);
        m_leftIntakeLauncher.configure(leftLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // FIX: Right launcher config — inverted=false (opposite direction to left)
        SparkMaxConfig rightLauncherConfig = new SparkMaxConfig();
        rightLauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        rightLauncherConfig.voltageCompensation(12);
        rightLauncherConfig.idleMode(IdleMode.kCoast);
        rightLauncherConfig.inverted(false);
        m_rightIntakeLauncher.configure(rightLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Set the speed of both intake/launcher rollers
    public void setIntakeLauncherRoller(double power) {
        m_leftIntakeLauncher.set(power);
        m_rightIntakeLauncher.set(power);
    }

    // Set the speed of the indexer (feeder) roller
    public void setFeederRoller(double power) {
        m_indexer.set(power);
    }

    // FIX: stop() now delegates to setters — no code duplication
    public void stop() {
        setIntakeLauncherRoller(0);
        setFeederRoller(0);
    }

    @Override
    public void periodic() {
        // FIX: SmartDashboard monitoring moved here (runs every loop)
        // Constants are shown for reference; live motor outputs for debugging
        SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
        SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
        SmartDashboard.putNumber("Launcher Output", m_leftIntakeLauncher.get());
        SmartDashboard.putNumber("Indexer Output", m_indexer.get());
    }
}