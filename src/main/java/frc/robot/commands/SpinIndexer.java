package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.Shooter;

public class SpinIndexer extends Command {
  private final Shooter m_shooter;

  /**
   * @param shooter The shooter subsystem
   * @param speed The motor speed
   */
  public SpinIndexer(Shooter shooter) {
    this.m_shooter = shooter;

    // Declare subsystem dependencies
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    // Optionally do something when the command starts
  }

  @Override
  public void execute() {
    // Set indexer motor to the desired speed
    m_shooter.setIndexerSpeed(-1);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the indexer when the command ends
    m_shooter.setIndexerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
