package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class ManipulateIntake extends Command {
  private final Intake m_intake;
  /**
   * @param shooter The shooter subsystem
   */
  public ManipulateIntake(Intake intake) {
    this.m_intake = intake;

    // Declare subsystem dependencies
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    // Optionally do something when the command starts
  }

  @Override
  public void execute() {
    // Set intake motor to the desired speed
    m_intake.setAngleDown();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the intake when the command ends
    m_intake.setDefaultAngle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
