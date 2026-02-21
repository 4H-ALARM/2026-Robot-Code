package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.Shooter;

public class SpinSpindexer extends Command {
  private final Shooter m_shooter;
  // private final double m_speed;

  /**
   * @param spindexer The spindexer subsystem
   * @param speed The motor speed
   */
  public SpinSpindexer(Shooter shooter) {
    this.m_shooter = shooter;

    // Declare subsystem dependencies
    addRequirements();
  }

  @Override
  public void initialize() {
    // Optional: do something once when the command starts
  }

  @Override
  public void execute() {
    // Set spindexer motor to the desired speed
    m_shooter.setSpindexerSpeed(1);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop spindexer when the command ends
    m_shooter.setSpindexerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    // Keeps running until cancelled/interrupted (e.g., button released)
    return false;
  }
}
