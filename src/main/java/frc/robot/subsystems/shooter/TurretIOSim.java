// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.constants.GenericConstants;
import frc.lib.constants.ShooterConstants;
import frc.lib.enums.TargetEnum;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class TurretIOSim implements TurretIO {
  private static final double kDtSeconds = 0.02;

  private final ProfiledPIDController turretController;
  private final SimpleMotorFeedforward turretFeedforward;
  private final LoggedMechanism2d turretMechanism;
  private final LoggedMechanismLigament2d turretLigament;
  private final LoggedMechanismLigament2d baseLigament;

  private Rotation2d motorPosition;
  private TargetEnum targetEnum;
  private Pose2d robotPose;
  private Pose2d turretPose;
  private Pose2d targetPose;
  private boolean neutralMode;
  private boolean unwinding;
  private boolean manualControl;
  private double manualVoltage;
  private double turretPositionDeg;
  private double turretVelocityDegPerSec;

  public TurretIOSim() {
    turretController =
        new ProfiledPIDController(
            ShooterConstants.turretkp,
            ShooterConstants.turretki,
            ShooterConstants.turretkd,
            new TrapezoidProfile.Constraints(
                ShooterConstants.turretMaxSpeed, ShooterConstants.turretMaxAccel));
    turretController.enableContinuousInput(-180.0, 180.0);
    turretController.setTolerance(ShooterConstants.turretTolerance);
    turretFeedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.turretks, ShooterConstants.turretkv, ShooterConstants.turretka);

    turretMechanism = new LoggedMechanism2d(1.0, 1.0);
    LoggedMechanismRoot2d root = turretMechanism.getRoot("TurretRoot", 0.5, 0.5);
    baseLigament =
        root.append(
            new LoggedMechanismLigament2d("TurretBase", ShooterConstants.turretPoseOffset, 0.0));
    turretLigament = baseLigament.append(new LoggedMechanismLigament2d("Turret", 0.2, 0.0));

    motorPosition = new Rotation2d();
    turretPositionDeg = 0.0;
    turretVelocityDegPerSec = 0.0;
    neutralMode = false;
    unwinding = false;
    manualControl = false;
    manualVoltage = 0.0;
    robotPose = new Pose2d();
    targetEnum = TargetEnum.HUB;
    targetPose = GenericConstants.HUB_POSE2D;
  }

  @Override
  public void changeTarget(TargetEnum target) {
    targetEnum = target;
    switch (this.targetEnum) {
      case HUB:
        this.targetPose = GenericConstants.HUB_POSE2D;
        break;
      case ALLIANCEZONELEFT:
        this.targetPose = GenericConstants.LEFTALLIANCE;
        break;
      case ALLIANCEZONECENTER:
        this.targetPose = GenericConstants.CENTERALLIANCE;
        break;
      case ALLIANCEZONERIGHT:
        this.targetPose = GenericConstants.RIGHTALLIANCE;
        break;
    }
  }

  @Override
  public void toggleNeutral() {
    neutralMode = !neutralMode;
  }

  @Override
  public void resetEncoder() {
    turretPositionDeg = 0.0;
    turretVelocityDegPerSec = 0.0;
    turretController.reset(turretPositionDeg);
  }

  @Override
  public void updateTuningValues() {
    turretController.setPID(
        ShooterConstants.turretkp, ShooterConstants.turretki, ShooterConstants.turretkd);
  }

  public Rotation2d getTargetTurretAngle(Pose2d pose) {
    double y = targetPose.getY() - pose.getY();
    double x = targetPose.getX() - pose.getX();
    double totalangleradians = Math.atan2(y, x);

    return Rotation2d.fromRadians(-(totalangleradians - pose.getRotation().getRadians()));
  }

  public Pose2d getTurretPose() {
    Translation2d offset =
        new Translation2d(ShooterConstants.turretPoseOffset, 0.0).rotateBy(robotPose.getRotation());
    Translation2d turretTranslation = robotPose.getTranslation().plus(offset);
    turretPose = new Pose2d(turretTranslation, robotPose.getRotation());
    Logger.recordOutput("Turret/turretPose", turretPose);
    return turretPose;
  }

  @Override
  public void turretPeriodic(Pose2d currentRobotPose) {
    this.robotPose = currentRobotPose;
    updateTuningValues();

    double targetAngleDeg = getTargetTurretAngle(getTurretPose()).getDegrees();
    boolean atPositiveLimit =
        Math.abs(turretPositionDeg - ShooterConstants.maxRotationPositive)
            <= ShooterConstants.turretTolerance;
    boolean atNegativeLimit =
        Math.abs(turretPositionDeg + ShooterConstants.maxRotationNegative)
            <= ShooterConstants.turretTolerance;

    if (atPositiveLimit || atNegativeLimit || unwinding) {
      targetAngleDeg = 0.0;
      unwinding = true;
      if (Math.abs(turretPositionDeg) <= ShooterConstants.turretTolerance) {
        unwinding = false;
      }
    }

    double appliedVolts;
    if (manualControl) {
      appliedVolts = manualVoltage;
    } else {
      turretController.setGoal(targetAngleDeg);
      double pidOutput = turretController.calculate(turretPositionDeg);
      TrapezoidProfile.State setpoint = turretController.getSetpoint();
      double ffVolts = turretFeedforward.calculate(setpoint.velocity);
      appliedVolts = pidOutput + ffVolts;
      turretPositionDeg = setpoint.position;
      turretVelocityDegPerSec = setpoint.velocity;
    }

    if (neutralMode) {
      appliedVolts = 0.0;
      turretVelocityDegPerSec = 0.0;
    }

    if (manualControl) {
      double accelDegPerSecSq = 0.0;
      if (ShooterConstants.turretka != 0.0) {
        double sign = Math.signum(turretVelocityDegPerSec);
        accelDegPerSecSq =
            (appliedVolts
                    - (ShooterConstants.turretks * sign)
                    - (ShooterConstants.turretkv * turretVelocityDegPerSec))
                / ShooterConstants.turretka;
      }
      if (ShooterConstants.turretMaxAccel > 0.0) {
        accelDegPerSecSq =
            MathUtil.clamp(
                accelDegPerSecSq,
                -ShooterConstants.turretMaxAccel,
                ShooterConstants.turretMaxAccel);
      }

      turretVelocityDegPerSec += accelDegPerSecSq * kDtSeconds;
      if (ShooterConstants.turretMaxSpeed > 0.0) {
        turretVelocityDegPerSec =
            MathUtil.clamp(
                turretVelocityDegPerSec,
                -ShooterConstants.turretMaxSpeed,
                ShooterConstants.turretMaxSpeed);
      }
      turretPositionDeg += turretVelocityDegPerSec * kDtSeconds;
    }
    if (ShooterConstants.maxRotationPositive > 0.0 || ShooterConstants.maxRotationNegative > 0.0) {
      turretPositionDeg =
          MathUtil.clamp(
              turretPositionDeg,
              -ShooterConstants.maxRotationNegative,
              ShooterConstants.maxRotationPositive);
    }

    motorPosition = Rotation2d.fromDegrees(turretPositionDeg * ShooterConstants.turretRatio);
    baseLigament.setAngle(0.0);
    turretLigament.setAngle(turretPositionDeg);
    Logger.recordOutput("Turret/Mechanism2d", turretMechanism);
  }

  @Override
  public void manualcontrol(double controllerInput) {
    manualControl = true;
    manualVoltage = MathUtil.clamp(controllerInput, -1.0, 1.0) * 12.0;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.isConnected = true;
    inputs.target = targetEnum;
    inputs.isReady =
        Math.abs(turretPositionDeg - getTargetTurretAngle(getTurretPose()).getDegrees())
            <= ShooterConstants.turretTolerance;
    inputs.robotRelativeAngleDegrees = motorPosition.getDegrees();
    inputs.turretRelativeAngleDegrees = motorPosition.getDegrees();
    Pose2d turretpose = getTurretPose();
    inputs.turretPoseReal =
        new Pose2d(turretpose.getX(), turretpose.getY(), Rotation2d.fromDegrees(turretPositionDeg));
    Pose3d turretpose3d = turretMechanism.generate3dMechanism().get(1);
    Pose3d turretpose3dtranslated =
        new Pose3d(
            turretpose3d.getTranslation().getX() + robotPose.getX(),
            turretpose3d.getTranslation().getY() + robotPose.getY(),
            turretpose3d.getZ(),
            turretpose3d.getRotation().rotateBy(new Rotation3d(Math.toRadians(90.0), 0.0, 0.0)));
    Logger.recordOutput("Turret/realpose3d", turretpose3dtranslated);
  }
}
