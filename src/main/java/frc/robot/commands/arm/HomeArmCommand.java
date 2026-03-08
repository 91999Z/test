package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class HomeArmCommand extends SequentialCommandGroup {
  public HomeArmCommand(ArmSubsystem arm) {
    setName("HomeArm");
    addRequirements(arm);

    addCommands(
        new InstantCommand(
            () -> {
              arm.enableSoftlimits(false);
              arm.setVolts(Constants.Arm.HOME_DRIVE_VOLTS);
            }),
        new WaitUntilCommand(arm::getReverseLimitSwitchPressed)
            .withTimeout(Constants.Arm.HOME_TIMEOUT_SECONDS),
        new InstantCommand(
            () -> {
              arm.setVolts(0.0);
              arm.resetArmEncoder();
              arm.setSoftLimits(
                  Constants.Arm.MAX_FORWARD_SOFT_LIMIT_DEG,
                  Constants.Arm.MAX_REVERSE_SOFT_LIMIT_DEG);
              arm.enableSoftlimits(true);
            }));
  }
}
