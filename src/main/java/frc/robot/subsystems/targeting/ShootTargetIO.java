// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.targeting.ShootTargetIO.ShootTargetIOInputs.AutoWinner;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public class ShootTargetIO extends SubsystemBase {
  final Translation3d m_defaultTarget;
  final boolean m_defaultfollowPhaseShift;
  Translation3d m_target;
  boolean m_followPhaseShift;

  // time before next phase is active where the hub will register scoring fuel.
  public static final double graceStartPeriod = 0;
  // time after current phase where the hub will register scoring fuel.
  public static final double graceEndPeriod = 3;

  @AutoLog
  public static class ShootTargetIOInputs {
    public Translation3d target;
    public boolean followPhaseShift;

    public enum AutoWinner {
      UNDETERMINED,
      RED_ALLIANCE,
      BLUE_ALLIANCE
    };

    public AutoWinner autoWinner;
    public boolean myHubActive;
    public double phaseTimeRemaining;
  }

  /**
   * Creates a new ShootTarget.
   *
   * <p>some targets like hub will be available or unavailable based on which specific phaseshift is
   * active./*
   */
  public ShootTargetIO(Translation3d defaultTarget, boolean followPhaseShift) {
    this.m_defaultTarget = defaultTarget;
    this.m_defaultfollowPhaseShift = followPhaseShift;

    this.resetTarget();
  }

  public void resetTarget() {
    setTarget(m_defaultTarget, m_defaultfollowPhaseShift);
  }

  public void setTarget(Translation3d target, boolean followPhaseShift) {
    this.m_target = target;
    this.m_followPhaseShift = followPhaseShift;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateInputs(ShootTargetIOInputs inputs) {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      inputs.myHubActive = false;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      inputs.myHubActive = true;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      inputs.myHubActive = false;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      inputs.myHubActive = true;
      inputs.autoWinner = AutoWinner.UNDETERMINED;
      return;
    }

    switch (gameData.charAt(0)) {
      case 'R' -> inputs.autoWinner = AutoWinner.RED_ALLIANCE;
      case 'B' -> inputs.autoWinner = AutoWinner.BLUE_ALLIANCE;
      default -> {
        // If we have invalid game data, assume hub is active.
        inputs.myHubActive = true;
        inputs.autoWinner = AutoWinner.UNDETERMINED;
        return;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> inputs.autoWinner == AutoWinner.BLUE_ALLIANCE;
          case Blue -> inputs.autoWinner == AutoWinner.RED_ALLIANCE;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      inputs.myHubActive = true;
      inputs.phaseTimeRemaining = matchTime - 130;
    } else if (matchTime > 105) {
      // Shift 1
      inputs.myHubActive = shift1Active;
      inputs.phaseTimeRemaining = matchTime - 105;

    } else if (matchTime > 80) {
      // Shift 2
      inputs.myHubActive = !shift1Active;
      inputs.phaseTimeRemaining = matchTime - 80;
    } else if (matchTime > 55) {
      // Shift 3
      inputs.myHubActive = shift1Active;
      inputs.phaseTimeRemaining = matchTime - 55;
    } else if (matchTime > 30) {
      // Shift 4
      inputs.myHubActive = !shift1Active;
      inputs.phaseTimeRemaining = matchTime - 30;
    } else {
      // End game, hub always active.
      inputs.myHubActive = true;
      inputs.phaseTimeRemaining = matchTime;
    }
  }
}
