// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.targeting.ShootTargetIO;
import frc.robot.subsystems.targeting.ShootTargetIO.ShootTargetIOInputs;

public class Shooter extends SubsystemBase {

  private ShooterIO shooter;
  private Drive drive;
  private IndexerIO indexer;
  private ShootTargetIO shootTarget;
  private ShootTargetIOInputs shootTargetInputs;

  /** FIX DO NOT WANT TO IMPORT A WHOLE DRIVE */
  public Shooter(ShooterIO shooter, Drive drive, IndexerIO indexer, ShootTargetIO shootTarget) {
    this.shooter = shooter;
    this.drive = drive;
    this.indexer = indexer;
    this.shootTarget = shootTarget;
  }

  @Override
  public void periodic() {
    shootTarget.updateInputs(shootTargetInputs);
  }

  public void spinShooter(double speed) {
    shooter.setShooterSpeed(speed);
  }

  public void stopShooter() {
    shooter.setShooterSpeed(0);
  }

  public void setHoodAngle(double hoodAngle) {
    shooter.setHoodAngle(hoodAngle);
  }

  public void setIndexerSpeed(double indexerSpeed) {
    indexer.setIndexerSpeed(indexerSpeed);
  }

  public void setTarget(Translation3d target, boolean targetAvailable) {
    shootTarget.setTarget(target, targetAvailable);
  }
}
