package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModuleSim m_frontLeft = new SwerveModuleSim();
  private final SwerveModuleSim m_frontRight = new SwerveModuleSim();
  private final SwerveModuleSim m_backLeft = new SwerveModuleSim();
  private final SwerveModuleSim m_backRight = new SwerveModuleSim();

  private final Field2d m_field = new Field2d();
  private final Timer m_timer = new Timer();

  private Rotation2d m_gyroYaw = new Rotation2d();
  private ChassisSpeeds m_lastRobotRelativeSpeeds = new ChassisSpeeds();

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          Constants.Drive.KINEMATICS, m_gyroYaw, getModulePositions(), new Pose2d());

  public DriveSubsystem() {
    m_timer.start();
    SmartDashboard.putData("Field", m_field);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    ChassisSpeeds targetSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, m_gyroYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

    m_lastRobotRelativeSpeeds = targetSpeeds;

    SwerveModuleState[] desiredStates = Constants.Drive.KINEMATICS.toSwerveModuleStates(targetSpeeds);
    setModuleStates(desiredStates);
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_frontLeft.reset();
    m_frontRight.reset();
    m_backLeft.reset();
    m_backRight.reset();
    m_gyroYaw = pose.getRotation();
    m_odometry.resetPosition(m_gyroYaw, getModulePositions(), pose);
  }

  private void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_LINEAR_SPEED_MPS);
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  @Override
  public void periodic() {
    double dt = m_timer.get();
    m_timer.reset();

    m_frontLeft.update(dt);
    m_frontRight.update(dt);
    m_backLeft.update(dt);
    m_backRight.update(dt);

    m_gyroYaw = m_gyroYaw.plus(new Rotation2d(m_lastRobotRelativeSpeeds.omegaRadiansPerSecond * dt));
    m_odometry.update(m_gyroYaw, getModulePositions());

    SmartDashboard.putNumber("Drive/X", getPose().getX());
    SmartDashboard.putNumber("Drive/Y", getPose().getY());
    SmartDashboard.putNumber("Drive/HeadingDeg", m_gyroYaw.getDegrees());
    m_field.setRobotPose(getPose());
  }
}
