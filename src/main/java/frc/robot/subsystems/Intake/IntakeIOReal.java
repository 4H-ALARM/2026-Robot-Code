// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Constants.IntakeConstants;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {
  public final TalonFX tiltLeader;
  public final TalonFX tiltFollower;
  public final TalonFX intakeMotor;

  private Slot0Configs pivotConfig;
  private MotionMagicConfigs pivotMotionMagicConfig;
  private Slot0Configs intakeConfig;
  private MotionMagicConfigs intakeMotionMagicConfig;
  private TalonFXConfiguration pivotMotorConfig;
  private TalonFXConfiguration intakeMotorConfig;

  private VelocityTorqueCurrentFOC intakeControl;
  private PositionTorqueCurrentFOC pivotControl;
  private Follower followerControl;

  private final LoggedMechanism2d intakeMechanism;
  private final LoggedMechanismLigament2d pivotLigament;

  private final LoggedTunableNumber pivotkp =
      new LoggedTunableNumber("Intake/pivotkp", IntakeConstants.pivotkp);
  private final LoggedTunableNumber pivotki =
      new LoggedTunableNumber("Intake/pivotki", IntakeConstants.pivotki);
  private final LoggedTunableNumber pivotkd =
      new LoggedTunableNumber("Intake/pivotkd", IntakeConstants.pivotkd);
  private final LoggedTunableNumber pivotks =
      new LoggedTunableNumber("Intake/pivotks", IntakeConstants.pivotks);
  private final LoggedTunableNumber pivotkv =
      new LoggedTunableNumber("Intake/pivotkv", IntakeConstants.pivotkv);
  private final LoggedTunableNumber pivotka =
      new LoggedTunableNumber("Intake/pivotka", IntakeConstants.pivotka);
  private final LoggedTunableNumber pivotMaxAccel =
      new LoggedTunableNumber("Intake/pivotMaxAccel", IntakeConstants.pivotMaxAccel);
  private final LoggedTunableNumber pivotMaxSpeed =
      new LoggedTunableNumber("Intake/pivotMaxSpeed", IntakeConstants.pivotMaxSpeed);
  private final LoggedTunableNumber pivotJerk =
      new LoggedTunableNumber("Intake/pivotJerk", IntakeConstants.pivotJerk);
  private final LoggedTunableNumber pivotRatio =
      new LoggedTunableNumber("Intake/pivotRatio", IntakeConstants.pivotRatio);

  private final LoggedTunableNumber intakekp =
      new LoggedTunableNumber("Intake/intakekp", IntakeConstants.intakekp);
  private final LoggedTunableNumber intakeki =
      new LoggedTunableNumber("Intake/intakeki", IntakeConstants.intakeki);
  private final LoggedTunableNumber intakekd =
      new LoggedTunableNumber("Intake/intakekd", IntakeConstants.intakekd);
  private final LoggedTunableNumber intakeks =
      new LoggedTunableNumber("Intake/intakeks", IntakeConstants.intakeks);
  private final LoggedTunableNumber intakekv =
      new LoggedTunableNumber("Intake/intakekv", IntakeConstants.intakekv);
  private final LoggedTunableNumber intakeka =
      new LoggedTunableNumber("Intake/intakeka", IntakeConstants.intakeka);
  private final LoggedTunableNumber intakeMaxAccel =
      new LoggedTunableNumber("Intake/intakeMaxAccel", IntakeConstants.intakeMaxAccel);
  private final LoggedTunableNumber intakeMaxSpeed =
      new LoggedTunableNumber("Intake/intakeMaxSpeed", IntakeConstants.intakeMaxSpeed);
  private final LoggedTunableNumber intakeJerk =
      new LoggedTunableNumber("Intake/intakeJerk", IntakeConstants.intakeJerk);

  public IntakeIOReal() {
    tiltLeader = new TalonFX(IntakeConstants.pivotLeaderID);
    tiltFollower = new TalonFX(IntakeConstants.pivotFollowerID);
    intakeMotor = new TalonFX(IntakeConstants.intakeMotorId);

    pivotConfig =
        new Slot0Configs()
            .withKP(pivotkp.get())
            .withKI(pivotki.get())
            .withKD(pivotkd.get())
            .withKA(pivotka.get())
            .withKS(pivotks.get())
            .withKV(pivotkv.get());

    pivotMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(pivotMaxAccel.get())
            .withMotionMagicCruiseVelocity(pivotMaxSpeed.get())
            .withMotionMagicJerk(pivotJerk.get());

    intakeConfig =
        new Slot0Configs()
            .withKP(intakekp.get())
            .withKI(intakeki.get())
            .withKD(intakekd.get())
            .withKA(intakeka.get())
            .withKS(intakeks.get())
            .withKV(intakekv.get());

    intakeMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(intakeMaxAccel.get())
            .withMotionMagicCruiseVelocity(intakeMaxSpeed.get())
            .withMotionMagicJerk(intakeJerk.get());

    pivotMotorConfig = new TalonFXConfiguration();
    pivotMotorConfig.Slot0 = pivotConfig;
    pivotMotorConfig.MotionMagic = pivotMotionMagicConfig;
    pivotMotorConfig.Feedback.SensorToMechanismRatio = pivotRatio.get();

    intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.Slot0 = intakeConfig;
    intakeMotorConfig.MotionMagic = intakeMotionMagicConfig;

    tiltLeader.getConfigurator().apply(pivotMotorConfig);
    tiltLeader.setPosition(0);
    tiltFollower.getConfigurator().apply(pivotMotorConfig);

    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    intakeControl = new VelocityTorqueCurrentFOC(0);
    pivotControl = new PositionTorqueCurrentFOC(0);
    followerControl = new Follower(IntakeConstants.pivotLeaderID, MotorAlignmentValue.Opposed);

    intakeMechanism = new LoggedMechanism2d(1.0, 1.0);
    LoggedMechanismRoot2d root = intakeMechanism.getRoot("IntakeRoot", 0.5, 0.5);
    pivotLigament = root.append(new LoggedMechanismLigament2d("IntakePivot", 0.3, 0.0));

    intakeMotor.setControl(intakeControl);
    tiltLeader.setControl(pivotControl);
    tiltFollower.setControl(followerControl);
  }

  @Override
  public void updateTuningValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          pivotConfig =
              new Slot0Configs()
                  .withKP(pivotkp.get())
                  .withKI(pivotki.get())
                  .withKD(pivotkd.get())
                  .withKA(pivotka.get())
                  .withKS(pivotks.get())
                  .withKV(pivotkv.get());
          tiltLeader.getConfigurator().apply(pivotConfig);
          tiltFollower.getConfigurator().apply(pivotConfig);
        },
        pivotkp,
        pivotki,
        pivotkd,
        pivotka,
        pivotks,
        pivotkv);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          pivotMotionMagicConfig =
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(pivotMaxAccel.get())
                  .withMotionMagicCruiseVelocity(pivotMaxSpeed.get())
                  .withMotionMagicJerk(pivotJerk.get());
          tiltLeader.getConfigurator().apply(pivotMotionMagicConfig);
          tiltFollower.getConfigurator().apply(pivotMotionMagicConfig);
        },
        pivotMaxAccel,
        pivotMaxSpeed,
        pivotJerk);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          pivotMotorConfig.Feedback.SensorToMechanismRatio = pivotRatio.get();
          tiltLeader.getConfigurator().apply(pivotMotorConfig);
          tiltFollower.getConfigurator().apply(pivotMotorConfig);
        },
        pivotRatio);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          intakeConfig =
              new Slot0Configs()
                  .withKP(intakekp.get())
                  .withKI(intakeki.get())
                  .withKD(intakekd.get())
                  .withKA(intakeka.get())
                  .withKS(intakeks.get())
                  .withKV(intakekv.get());
          intakeMotor.getConfigurator().apply(intakeConfig);
        },
        intakekp,
        intakeki,
        intakekd,
        intakeka,
        intakeks,
        intakekv);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          intakeMotionMagicConfig =
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(intakeMaxAccel.get())
                  .withMotionMagicCruiseVelocity(intakeMaxSpeed.get())
                  .withMotionMagicJerk(intakeJerk.get());
          intakeMotor.getConfigurator().apply(intakeMotionMagicConfig);
        },
        intakeMaxAccel,
        intakeMaxSpeed,
        intakeJerk);
  }

  @Override
  public void setAngle(Rotation2d targetRotation) {
    tiltLeader.setControl(pivotControl.withPosition(targetRotation.getDegrees()));
  }

  @Override
  public void setSpeed(double speed) {
    intakeMotor.setControl(intakeControl.withVelocity(speed));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected = intakeMotor.isConnected();
    inputs.pivotLeadMotorConnected = tiltLeader.isConnected();
    inputs.pivotFollowerMotorConnected = tiltFollower.isConnected();

    inputs.angleMotorCounts = tiltLeader.getPosition().getValueAsDouble();
    inputs.speed = intakeMotor.getVelocity().getValueAsDouble();
    pivotLigament.setAngle(inputs.angleMotorCounts);
    Logger.recordOutput("Intake/mechanism2d", intakeMechanism);
    inputs.pivotPose3d = intakeMechanism.generate3dMechanism().get(1);
  }
}
