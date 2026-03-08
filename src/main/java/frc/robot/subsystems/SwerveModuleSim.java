package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleSim {
  private SwerveModuleState m_state = new SwerveModuleState();
  private double m_distanceMeters = 0.0;

  public void setDesiredState(SwerveModuleState desiredState) {
    m_state = SwerveModuleState.optimize(desiredState, m_state.angle);
  }

  public void update(double dtSeconds) {
    m_distanceMeters += m_state.speedMetersPerSecond * dtSeconds;
  }

  public SwerveModuleState getState() {
    return m_state;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_distanceMeters, m_state.angle);
  }

  public void reset() {
    m_state = new SwerveModuleState(0.0, new Rotation2d());
    m_distanceMeters = 0.0;
  }
}
