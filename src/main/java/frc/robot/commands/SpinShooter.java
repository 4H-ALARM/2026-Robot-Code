package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.Shooter;

public class SpinShooter extends Command {
  private final Shooter m_shooter;
  private final double m_speed;

  /**
   * @param shooter The shooter subsystem
   * @param speed The motor speed
   */
  public SpinShooter(Shooter shooter, double speed) {
    this.m_shooter = shooter;
    this.m_speed = speed;

    // Declare subsystem dependencies
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Optionally do something when the command starts
  }

  @Override
  public void execute() {
    // Set shooter motor to the desired speed
    m_shooter.spinShooter(m_speed);
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
