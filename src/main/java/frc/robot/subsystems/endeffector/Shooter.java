// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.GenericConstants;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase {
  // Lookup table values must stay aligned by index.
  // Distance is meters from robot pose to hub pose.
  private static final double[] SHOOTER_DISTANCE_LOOKUP_METERS = {0.889, 1.0, 2.0, 4.0, 5.0};
  private static final double[] SHOOTER_SPEED_LOOKUP = {-32.5, -35, -40.0, -42.5, -45.0};

  private TurretIOInputsAutoLogged turretInputs;
  private ShooterIOInputsAutoLogged shooterInputs;
  private TurretIO turret;
  private ShooterIO shooter;
  private Drive drive;
  private IndexerIO indexer;
  private SpindexerIO m_spindexer;

  /** FIX DO NOT WANT TO IMPORT A WHOLE DRIVE */
  public Shooter(
      ShooterIO shooter, TurretIO turret, Drive drive, IndexerIO indexer, SpindexerIO spindexer) {
    this.turretInputs = new TurretIOInputsAutoLogged();
    this.shooterInputs = new ShooterIOInputsAutoLogged();
    this.turret = turret;
    this.shooter = shooter;
    this.drive = drive;
    this.indexer = indexer;
    this.m_spindexer = spindexer;
  }

  @Override
  public void periodic() {
    // this.turret.updateInputs(turretInputs);
    // this.turret.turretPeriodic(drive.getPose());
    this.shooter.updateInputs(shooterInputs);
  }

  public Rotation2d getTargetTurretAngle(Pose2d pose) {
    return turret.getTargetTurretAngle(pose);
  }

  public void spinShooter(double speed) {
    shooter.setShooterSpeed(speed);
  }

  public void setHoodAngle(double angle) {
    shooter.setHoodAngle(angle);
  }

  public void setIndexerSpeed(double indexerSpeed) {
    indexer.setIndexerSpeed(indexerSpeed);
  }

  public void setSpindexerSpeed(double speed) {
    m_spindexer.setSpindexerSpeed(speed);
  }

  /** Returns robot distance to hub in meters based on current drive pose. */
  public double getDistanceToHubMeters() {
    return drive
        .getPose()
        .getTranslation()
        .getDistance(GenericConstants.HUB_POSE2D.getTranslation());
  }

  /**
   * Calculates shooter speed using linear interpolation over the lookup table.
   *
   * <p>If distance is outside the table, this clamps to the closest endpoint speed.
   */
  public double calculateShooterSpeedFromDistance(double distanceMeters) {
    if (SHOOTER_DISTANCE_LOOKUP_METERS.length != SHOOTER_SPEED_LOOKUP.length
        || SHOOTER_DISTANCE_LOOKUP_METERS.length == 0) {
      throw new IllegalStateException("Shooter lookup tables must be non-empty and equal length.");
    }

    if (distanceMeters <= SHOOTER_DISTANCE_LOOKUP_METERS[0]) {
      return SHOOTER_SPEED_LOOKUP[0];
    }

    int lastIndex = SHOOTER_DISTANCE_LOOKUP_METERS.length - 1;
    if (distanceMeters >= SHOOTER_DISTANCE_LOOKUP_METERS[lastIndex]) {
      return SHOOTER_SPEED_LOOKUP[lastIndex];
    }

    for (int i = 0; i < lastIndex; i++) {
      double lowerDistance = SHOOTER_DISTANCE_LOOKUP_METERS[i];
      double upperDistance = SHOOTER_DISTANCE_LOOKUP_METERS[i + 1];
      if (distanceMeters >= lowerDistance && distanceMeters <= upperDistance) {
        double lowerSpeed = SHOOTER_SPEED_LOOKUP[i];
        double upperSpeed = SHOOTER_SPEED_LOOKUP[i + 1];
        double fraction = (distanceMeters - lowerDistance) / (upperDistance - lowerDistance);
        return lowerSpeed + fraction * (upperSpeed - lowerSpeed);
      }
    }

    return SHOOTER_SPEED_LOOKUP[lastIndex];
  }

  /** Calculates shooter speed from current drive pose and applies it. */
  public void spinShooterFromPoseDistance() {
    spinShooter(calculateShooterSpeedFromDistance(getDistanceToHubMeters()));
  }
}
