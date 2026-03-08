package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDriveCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_xInput;
  private final DoubleSupplier m_yInput;
  private final DoubleSupplier m_rotInput;
  private final BooleanSupplier m_fieldRelative;

  public SwerveDriveCommand(
      DriveSubsystem drive,
      DoubleSupplier xInput,
      DoubleSupplier yInput,
      DoubleSupplier rotInput,
      BooleanSupplier fieldRelative) {
    m_drive = drive;
    m_xInput = xInput;
    m_yInput = yInput;
    m_rotInput = rotInput;
    m_fieldRelative = fieldRelative;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double x =
        -MathUtil.applyDeadband(m_xInput.getAsDouble(), Constants.OI.DEADBAND)
            * Constants.Drive.MAX_LINEAR_SPEED_MPS;
    double y =
        -MathUtil.applyDeadband(m_yInput.getAsDouble(), Constants.OI.DEADBAND)
            * Constants.Drive.MAX_LINEAR_SPEED_MPS;
    double rot =
        -MathUtil.applyDeadband(m_rotInput.getAsDouble(), Constants.OI.DEADBAND)
            * Constants.Drive.MAX_ANGULAR_SPEED_RADPS;

    m_drive.drive(x, y, rot, m_fieldRelative.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}
