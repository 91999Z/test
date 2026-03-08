package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class OI {
    public static final int DRIVER_PORT = 0;
    public static final int ARM_PORT = 1;
    public static final double DEADBAND = 0.08;
  }

  public static final class Drive {
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.0);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.0);
    public static final double MAX_LINEAR_SPEED_MPS = 4.0;
    public static final double MAX_ANGULAR_SPEED_RADPS = Math.PI;

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),   // FL
            new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),  // FR
            new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),  // BL
            new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0)  // BR
        );
  }

  public static final class Arm {
    public static final double START_ANGLE_DEG = 40.0;
    public static final double SIM_DEGREES_PER_VOLT_PER_SEC = 12.0;
    public static final double MAX_FORWARD_SOFT_LIMIT_DEG = 93.0;
    public static final double MAX_REVERSE_SOFT_LIMIT_DEG = 0.5;
    public static final double HOME_DRIVE_VOLTS = -1.0;
    public static final double MANUAL_UP_VOLTS = 2.0;
    public static final double MANUAL_DOWN_VOLTS = -2.0;
    public static final double HOME_TIMEOUT_SECONDS = 4.0;
  }
}
