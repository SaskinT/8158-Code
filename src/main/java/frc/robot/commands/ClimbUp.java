package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

import static frc.robot.Constants.ClimbConstatns.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbUp extends Command {
  /** Creates a new Climber. */

  ClimbSubsystem climbSubsystem;

  public ClimbUp(ClimbSubsystem climbSubsystem) {
    addRequirements(climbSubsystem);
    this.climbSubsystem = climbSubsystem;
  }

  // Called when the command is initially scheduled. Set the climber to the
  // appropriate values for unclimbing
  @Override
  public void initialize() {
    climbSubsystem
        .setClimber(CLIMBER_MOTOR_UP_PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the climber
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}