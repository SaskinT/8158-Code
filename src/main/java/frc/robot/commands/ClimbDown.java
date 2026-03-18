package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import static frc.robot.Constants.ClimbConstants.*; // Hata burada çözüldü

public class ClimbDown extends Command {
  private final ClimbSubsystem climbSubsystem;

  public ClimbDown(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.setClimber(CLIMBER_MOTOR_DOWN_PERCENT);
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}