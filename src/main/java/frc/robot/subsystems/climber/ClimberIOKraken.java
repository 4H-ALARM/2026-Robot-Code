// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.climber.ClimberIO;
import frc.lib.Constants.ClimberConstants;

/** Add your docs here. */
public class ClimberIOKraken implements ClimberIO{

    private  TalonFX m_motor;
    private Slot0Configs m_pIDConfigs;
    private PositionVoltage m_requestedVoltage;
    private ClimberConstants m_constants;

    public ClimberIOKraken() {
        m_motor = new TalonFX(-1); 
        m_pIDConfigs = new Slot0Configs()
        .withKP(m_constants.climberkp)
        .withKI(m_constants.climberki)
        .withKD(m_constants.climberkd);
        m_requestedVoltage = new PositionVoltage(0).withSlot(0);
        m_motor.getConfigurator().apply(m_pIDConfigs);
    }

    public void moveArm(double speed){
        if(speed > 1){
            speed = 1;
        }
        if(speed < -1){
            speed = -1;
        }

        m_motor.set(speed);
    }
    public void setArmPosition(double positionInches, ClimberIOInputs imputs){
        double targetPositionInches = positionInches - imputs.currentLengthInches;

        double targetPositionRotations = targetPositionInches/ m_constants.climberRotationToInches;

        m_motor.setControl(m_requestedVoltage.withPosition(targetPositionRotations));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        // need to test how many encoder units equals one full motor rotation before setting the inches extended.
    }
}
