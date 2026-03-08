package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.arm.HomeArmCommand;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  private final CommandJoystick m_driveStick = new CommandJoystick(Constants.OI.DRIVER_PORT);
  private final CommandJoystick m_armStick = new CommandJoystick(Constants.OI.ARM_PORT);

  public RobotContainer() {
    m_drive.setDefaultCommand(
        new SwerveDriveCommand(
            m_drive,
            () -> m_driveStick.getRawAxis(1), // Forward/back
            () -> m_driveStick.getRawAxis(0), // Left/right strafe
            () -> m_driveStick.getRawAxis(2), // Rotate
            () -> !m_driveStick.button(6).getAsBoolean() // Hold button 6 for robot-relative
            ));

    configureBindings();
  }

  private void configureBindings() {
    // Arm manual up while held (button 5), down while held (button 3), stop on release.
    m_armStick.button(5)
        .whileTrue(new RunCommand(() -> m_arm.setVolts(Constants.Arm.MANUAL_UP_VOLTS), m_arm))
        .onFalse(new InstantCommand(m_arm::stop, m_arm));

    m_armStick.button(3)
        .whileTrue(new RunCommand(() -> m_arm.setVolts(Constants.Arm.MANUAL_DOWN_VOLTS), m_arm))
        .onFalse(new InstantCommand(m_arm::stop, m_arm));

    // Home arm sequence on button 1.
    m_armStick.button(1).onTrue(new HomeArmCommand(m_arm));

    // Emergency stop on button 2.
    m_armStick.button(2).onTrue(new InstantCommand(m_arm::stop, m_arm));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(() -> {}, m_drive, m_arm);
  }
}
