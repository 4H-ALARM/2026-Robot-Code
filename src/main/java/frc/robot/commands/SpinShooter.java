package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.Shooter;

public class SpinShooter extends Command {
  private final Shooter m_shooter;

  /**
   * @param shooter The shooter subsystem
   */
  public SpinShooter(Shooter shooter) {
    this.m_shooter = shooter;

    // Declare subsystem dependencies
    addRequirements();
  }

  @Override
  public void initialize() {
    // Optionally do something when the command starts
  }

  @Override
  public void execute() {
    m_shooter.spinShooterFromPoseDistance();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter when the command ends
    m_shooter.spinShooter(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
