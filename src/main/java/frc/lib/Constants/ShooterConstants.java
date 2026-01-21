// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.constants;

/** Shooter constants for the turret, the hood and the shooter itself */
public class ShooterConstants {

  // Device IDs
  public static final int turretMotorID = 1;
  public static final int shooterMotorID = 2;
  public static final int hoodMotorID = 3;
  public static final int neutralToggleButtonPort = 0;
  public static final int encoderResetButtonPort = 1;

  // Turret Tuning Constants
  public static final double turretkp = 1.0;
  public static final double turretki = 0.0;
  public static final double turretkd = 0.0;
  public static final double turretks = 0.0;
  public static final double turretkv = 0.0;
  public static final double turretka = 1.0;
  public static final double turretMaxAccel = 0.0;
  public static final double turretMaxSpeed = 0.0;
  public static final double turretJerk = 0.0;
  public static final double turretRatio = 1.0;
  public static final double maxRotation = 360.0;
  public static final double turretTolerance = 5;
  public static final double turretPoseOffset = 0.3175;

  public static final double shooterkp = 1.0;
  public static final double shooterki = 0.0;
  public static final double shooterkd = 0.0;
  public static final double shooterks = 0.0;
  public static final double shooterkv = 0.0;
  public static final double shooterka = 1.0;
  public static final double shooterMaxAccel = 0.0;
  public static final double shooterMaxSpeed = 0.0;
  public static final double shooterJerk = 0.0;

  public static final double hoodkp = 1.0;
  public static final double hoodki = 0.0;
  public static final double hoodkd = 0.0;
  public static final double hoodks = 0.0;
  public static final double hoodkv = 0.0;
  public static final double hoodka = 1.0;
  public static final double hoodkg = 1.0;
  public static final double hoodMaxAccel = 0.0;
  public static final double hoodMaxSpeed = 0.0;
  public static final double hoodJerk = 0.0;
}
