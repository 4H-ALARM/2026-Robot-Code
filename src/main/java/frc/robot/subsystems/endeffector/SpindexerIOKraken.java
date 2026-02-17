package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.Constants.IndexerConstants;
import frc.robot.subsystems.endeffector.SpindexerIO;

public class SpindexerIOKraken implements SpindexerIO {

  private TalonFX m_motor;
  private IndexerConstants m_constants;
  private PIDController m_pid;

  public SpindexerIOKraken() {
    m_motor = new TalonFX(m_constants.spindexerMotorID, "Turret");
    m_pid =
        new PIDController(
            m_constants.spindexerkp, m_constants.spindexerki, m_constants.spindexerkd);
    m_pid.setTolerance(5);
  }

  public void setSpindexerSpeed(double speed) {
    m_motor.set(speed);
  }

  public void activateMotor() {
    m_pid.reset();
    // m_motor.set(m_pid);

  }

  public void stopMotor() {
    m_motor.stopMotor();
  }

  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.isSpindexerMotorConnected = m_motor.isConnected();
    inputs.speed = m_motor.getVelocity().getValueAsDouble();
  }
}
