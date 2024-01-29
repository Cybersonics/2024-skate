package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class DoNothingCommand extends Command {

  // Instant end the command so nothing happens.
  @Override
  public boolean isFinished() {
    return true;
  }
}