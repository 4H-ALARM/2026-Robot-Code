// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
  private final ProfiledPIDController pivotController;
  private final SimpleMotorFeedforward pivotFeedforward;
  private final LoggedMechanism2d intakeMechanism;
  private final LoggedMechanismLigament2d pivotLigament;

  private double targetAngleDeg;
  private double pivotPositionDeg;
  private double pivotVelocityDegPerSec;
  private double intakeSpeed;

  public IntakeIOSim() {
    pivotController =
        new ProfiledPIDController(
            IntakeConstants.pivotkp,
            IntakeConstants.pivotki,
            IntakeConstants.pivotkd,
            new TrapezoidProfile.Constraints(
                IntakeConstants.pivotMaxSpeed, IntakeConstants.pivotMaxAccel));
    pivotFeedforward =
        new SimpleMotorFeedforward(
            IntakeConstants.pivotks, IntakeConstants.pivotkv, IntakeConstants.pivotka);

    intakeMechanism = new LoggedMechanism2d(1.0, 1.0);
    LoggedMechanismRoot2d root = intakeMechanism.getRoot("IntakeRoot", 0.5, 0.5);
    pivotLigament = root.append(new LoggedMechanismLigament2d("IntakePivot", 0.3, 0.0));

    targetAngleDeg = 0.0;
    pivotPositionDeg = 0.0;
    pivotVelocityDegPerSec = 0.0;
    intakeSpeed = 0.0;
  }

  @Override
  public void setAngle(Rotation2d targetRotation) {
    targetAngleDeg = targetRotation.getDegrees();
  }

  @Override
  public void setSpeed(double speed) {
    intakeSpeed = speed;
  }

  @Override
  public void updateTuningValues() {
    pivotController.setPID(
        IntakeConstants.pivotkp, IntakeConstants.pivotki, IntakeConstants.pivotkd);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    updateTuningValues();
    pivotController.setGoal(targetAngleDeg);
    pivotController.calculate(pivotPositionDeg);
    TrapezoidProfile.State setpoint = pivotController.getSetpoint();
    pivotFeedforward.calculate(setpoint.velocity);

    pivotPositionDeg = setpoint.position;
    pivotVelocityDegPerSec = setpoint.velocity;

    pivotLigament.setAngle(pivotPositionDeg);
    Logger.recordOutput("Intake/mechanism2d", intakeMechanism);
    Pose3d pivotPose3d = intakeMechanism.generate3dMechanism().get(0);
    inputs.pivotPose3d = pivotPose3d;

    inputs.intakeMotorConnected = true;
    inputs.pivotLeadMotorConnected = true;
    inputs.pivotFollowerMotorConnected = true;
    inputs.angleMotorCounts = pivotPositionDeg;
    inputs.speed = intakeSpeed;
  }
}
