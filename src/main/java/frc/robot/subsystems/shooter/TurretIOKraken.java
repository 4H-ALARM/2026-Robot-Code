// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.constants.GenericConstants;
import frc.lib.constants.ShooterConstants;
import frc.lib.enums.TargetEnum;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.ToggleHandler;

/** Add your docs here. */
public class TurretIOKraken implements TurretIO{
    private final TalonFX positionMotor;
    private Rotation2d motorPosition;
    private TargetEnum targetEnum;
    private Pose2d robotPose;
    private Pose2d targetPose;
    private PositionVoltage control;
    private DigitalInput neutralButton;
    private DigitalInput resetButton;
    private ToggleHandler neutralToggle;
    private Slot0Configs turretConfig;
    private MotionMagicConfigs turretMotionMagicConfig;
    private boolean unwinding;

    private final LoggedTunableNumber turretkp =
        new LoggedTunableNumber("Turret/kp", ShooterConstants.turretkp);
    private final LoggedTunableNumber turretki =
        new LoggedTunableNumber("Turret/ki", ShooterConstants.turretki);
    private final LoggedTunableNumber turretkd =
        new LoggedTunableNumber("Turret/kd", ShooterConstants.turretkd);
    private final LoggedTunableNumber turretka =
        new LoggedTunableNumber("Turret/ka", ShooterConstants.turretka);
    private final LoggedTunableNumber turretks =
        new LoggedTunableNumber("Turret/ks", ShooterConstants.turretks);
    private final LoggedTunableNumber turretkv =
        new LoggedTunableNumber("Turret/kv", ShooterConstants.turretkv);
    private final LoggedTunableNumber turretMaxAccel =
        new LoggedTunableNumber("Turret/maxAccel", ShooterConstants.turretMaxAccel);
    private final LoggedTunableNumber hoodMaxSpeed =
        new LoggedTunableNumber("Turret/hoodMaxSpeed", ShooterConstants.hoodMaxSpeed);
    private final LoggedTunableNumber turretJerk =
        new LoggedTunableNumber("Turret/jerk", ShooterConstants.turretJerk);
    
    
    public TurretIOKraken() {
        turretConfig = new Slot0Configs()
            .withKP(turretkp.get())
            .withKI(turretki.get())
            .withKD(turretkd.get())
            .withKA(turretka.get())
            .withKS(turretks.get())
            .withKV(turretkv.get());

        turretMotionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicAcceleration(turretMaxAccel.get())
            .withMotionMagicCruiseVelocity(hoodMaxSpeed.get())
            .withMotionMagicJerk(turretJerk.get());

        positionMotor = new TalonFX(ShooterConstants.turretMotorID);
        positionMotor.getConfigurator().apply(turretConfig);
        positionMotor.getConfigurator().apply(turretMotionMagicConfig);
        positionMotor.setPosition(0);
        positionMotor.setNeutralMode(NeutralModeValue.Brake);
        control = new PositionVoltage(0).withEnableFOC(true);

        neutralButton = new DigitalInput(ShooterConstants.neutralToggleButtonPort);
        resetButton = new DigitalInput(ShooterConstants.encoderResetButtonPort);
        neutralToggle = new ToggleHandler("Turret/NeutralModeToggle");
        unwinding = false;


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
        if (neutralButton.get() == true ) {
            neutralToggle.toggle();
        }
    }


    @Override
    public void resetEncoder() {
        if (resetButton.get() ==true) {
            positionMotor.setPosition(0);
        }
    }

    @Override
    public void updateTuningValues() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                turretConfig = new Slot0Configs()
                    .withKP(turretkp.get())
                    .withKI(turretki.get())
                    .withKD(turretkd.get())
                    .withKA(turretka.get())
                    .withKS(turretks.get())
                    .withKV(turretkv.get());
                positionMotor.getConfigurator().apply(turretConfig);
            },
            turretkp,
            turretki,
            turretkd,
            turretka,
            turretks,
            turretkv);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                turretMotionMagicConfig = new MotionMagicConfigs()
                    .withMotionMagicAcceleration(turretMaxAccel.get())
                    .withMotionMagicCruiseVelocity(hoodMaxSpeed.get())
                    .withMotionMagicJerk(turretJerk.get());
                positionMotor.getConfigurator().apply(turretMotionMagicConfig);
            },
            turretMaxAccel,
            hoodMaxSpeed,
            turretJerk);
    }

    private Rotation2d getTargetTurretAngle() {
        double y = targetPose.getY()-robotPose.getY();
        double x = targetPose.getX()-robotPose.getX();
        double totalangleradians = Math.atan(y/x);


        return Rotation2d.fromRadians(totalangleradians-robotPose.getRotation().getRadians());
    }

    @Override
    public void turretPeriodic(Pose2d currentRobotPose) {
        this.robotPose = currentRobotPose;

        if (
            positionMotor.getPosition().isNear(ShooterConstants.maxRotation/2, ShooterConstants.turretTolerance) 
            || 
            positionMotor.getPosition().isNear(-ShooterConstants.maxRotation/2, ShooterConstants.turretTolerance)
            ||
            unwinding ==true ) {
                positionMotor.setControl(control.withPosition(0).withEnableFOC(true));
                unwinding = true;
                if (positionMotor.getPosition().getValueAsDouble() ==0) {
                    unwinding = false;
                }
            }
        else {
            positionMotor.setControl(control.withPosition(getTargetTurretAngle().getDegrees()).withEnableFOC(true));
        }


        if (neutralToggle.get() == true) {
            positionMotor.setNeutralMode(NeutralModeValue.Coast);
        }
        if (neutralToggle.get() == false) {
            positionMotor.setNeutralMode(NeutralModeValue.Brake);
        }

    }



    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.isConnected = positionMotor.isConnected();
        inputs.target = targetEnum;
        inputs.isReady = positionMotor.getPosition().isNear(getTargetTurretAngle().getDegrees(), ShooterConstants.turretTolerance);
        inputs.robotRelativeAngleDegrees = positionMotor.getPosition().getValueAsDouble();
        inputs.turretRelativeAngleDegrees = positionMotor.getPosition().getValueAsDouble();
    }

}
