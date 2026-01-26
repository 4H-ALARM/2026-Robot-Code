package frc.lib.Constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

  public static final int pivotLeaderID = 0;
  public static final int pivotFollowerID = 1;
  public static final int intakeMotorId = 2;

  public static final Rotation2d downRotation = new Rotation2d();
  public static final Rotation2d upRotation = new Rotation2d();
  public static final double intakeVelocity = 0;

  public static final double pivotkp = 1.0;
  public static final double pivotki = 0.0;
  public static final double pivotkd = 0.0;
  public static final double pivotks = 0.0;
  public static final double pivotkv = 0.0;
  public static final double pivotka = 1.0;
  public static final double pivotMaxAccel = 50.0;
  public static final double pivotMaxSpeed = 50.0;
  public static final double pivotJerk = 50.0;
  public static final double pivotRatio = 11 / 60;

  public static final double intakekp = 1.0;
  public static final double intakeki = 0.0;
  public static final double intakekd = 0.0;
  public static final double intakeks = 0.0;
  public static final double intakekv = 0.0;
  public static final double intakeka = 1.0;
  public static final double intakeMaxAccel = 50.0;
  public static final double intakeMaxSpeed = 50.0;
  public static final double intakeJerk = 50.0;
}
