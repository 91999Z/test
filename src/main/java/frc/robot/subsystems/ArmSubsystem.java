package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private boolean m_softLimitsEnabled = true;
  private double m_forwardSoftLimitDeg = Constants.Arm.MAX_FORWARD_SOFT_LIMIT_DEG;
  private double m_reverseSoftLimitDeg = Constants.Arm.MAX_REVERSE_SOFT_LIMIT_DEG;

  private double m_appliedVolts = 0.0;
  private double m_angleDeg = Constants.Arm.START_ANGLE_DEG;

  private final Timer m_timer = new Timer();

  public ArmSubsystem() {
    m_timer.start();
  }

  public void enableSoftlimits(boolean enabled) {
    m_softLimitsEnabled = enabled;
  }

  public void setSoftLimits(double forwardDeg, double reverseDeg) {
    m_forwardSoftLimitDeg = forwardDeg;
    m_reverseSoftLimitDeg = reverseDeg;
  }

  public void setVolts(double volts) {
    double commanded = MathUtil.clamp(volts, -12.0, 12.0);

    if (m_softLimitsEnabled) {
      if (commanded > 0.0 && m_angleDeg >= m_forwardSoftLimitDeg) {
        commanded = 0.0;
      }

      if (commanded < 0.0 && m_angleDeg <= m_reverseSoftLimitDeg) {
        commanded = 0.0;
      }
    }

    m_appliedVolts = commanded;
  }

  public void stop() {
    setVolts(0.0);
  }

  public boolean getReverseLimitSwitchPressed() {
    return m_angleDeg <= m_reverseSoftLimitDeg;
  }

  public void resetArmEncoder() {
    m_angleDeg = 0.0;
  }

  public double getAngleDeg() {
    return m_angleDeg;
  }

  public double getAppliedVolts() {
    return m_appliedVolts;
  }

  @Override
  public void periodic() {
    double dt = m_timer.get();
    m_timer.reset();

    m_angleDeg += m_appliedVolts * Constants.Arm.SIM_DEGREES_PER_VOLT_PER_SEC * dt;
    m_angleDeg = Math.max(-5.0, m_angleDeg);

    SmartDashboard.putNumber("Arm/AngleDeg", m_angleDeg);
    SmartDashboard.putNumber("Arm/AppliedVolts", m_appliedVolts);
    SmartDashboard.putBoolean("Arm/ReverseLimitPressed", getReverseLimitSwitchPressed());
    SmartDashboard.putBoolean("Arm/SoftLimitsEnabled", m_softLimitsEnabled);
    SmartDashboard.putNumber("Arm/ForwardSoftLimitDeg", m_forwardSoftLimitDeg);
    SmartDashboard.putNumber("Arm/ReverseSoftLimitDeg", m_reverseSoftLimitDeg);
  }
}
