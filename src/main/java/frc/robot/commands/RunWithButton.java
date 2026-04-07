// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunWithButton extends Command {

  Trigger m_button;
  Command m_command;
  boolean m_expectedState;
  /** Creates a new runWithButton. */
  public RunWithButton(
    Trigger button,
    Command command,
    boolean expectedState
    ) {


    // Use addRequirements() here to declare subsystem dependencies.
    m_button = button;
    m_command = command;
    m_expectedState = expectedState;
    addRequirements(command.getRequirements());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished() || m_button.getAsBoolean() != m_expectedState;
  }

  public static RunWithButton runWhileButtonPressed(Trigger trigger, Command command) {
    return new RunWithButton(trigger, command, true);
  }

  public static RunWithButton runWhileButtonReleased(Trigger trigger, Command command) {
    return new RunWithButton(trigger, command, false);
  }
}
