// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase {

  private TurretIOInputsAutoLogged turretInputs;
  private TurretIO turret;
  private ShooterIO shooter;
  private Drive drive;

  /** FIX DO NOT WANT TO IMPORT A WHOLE DRIVE */
  public Shooter(ShooterIO shooter, TurretIO turret, Drive drive) {
    this.turretInputs = new TurretIOInputsAutoLogged();
    this.turret = turret;
    this.shooter = shooter;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    this.turret.updateInputs(turretInputs);
    this.turret.turretPeriodic(drive.getPose());
  }

  public Rotation2d getTargetTurretAngle(Pose2d pose) {
    return turret.getTargetTurretAngle(pose);
  }
}
