package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstatns.*;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;

  public ClimbSubsystem() {

    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);

  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    climberMotor.set(power);
  }

  // A method to stop the climber
  public void stop() {
    climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}