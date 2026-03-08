# Sim-Only FRC Practice Robot (Swerve + Realistic Arm)

This workspace now contains Java source files for a command-based practice robot:

- Simulated 4-module swerve drive
- Simulated arm with soft limits
- Arm homing command with reverse limit switch and timeout safety
- Keyboard-style joystick control via Driver Station simulation

## Files

- `src/main/java/frc/robot/subsystems/DriveSubsystem.java`
- `src/main/java/frc/robot/subsystems/SwerveModuleSim.java`
- `src/main/java/frc/robot/subsystems/ArmSubsystem.java`
- `src/main/java/frc/robot/commands/drive/SwerveDriveCommand.java`
- `src/main/java/frc/robot/commands/arm/HomeArmCommand.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/Constants.java`

## How to run in WPILib

1. Create a new WPILib Command-Based Java project in this folder (or copy these files into that project).
2. Build and launch simulation from WPILib VS Code.
3. In Driver Station sim, map keyboard to joystick ports:
   - Port 0 for swerve drive
   - Port 1 for arm controls
4. Open SmartDashboard/Shuffleboard to watch:
   - `Field`
   - `Drive/X`, `Drive/Y`, `Drive/HeadingDeg`
   - `Arm/AngleDeg`, `Arm/ReverseLimitPressed`, and soft limit values

## Suggested keyboard mapping in sim

Exact mapping depends on Driver Station keyboard profile, but these bindings are expected:

- Drive joystick axes:
  - Axis 1: forward/back
  - Axis 0: strafe left/right
  - Axis 2: rotate
  - Button 6 held: robot-relative driving (release for field-relative)
- Arm joystick buttons:
  - Button 5: arm up (hold)
  - Button 3: arm down (hold)
  - Button 1: run home sequence
  - Button 2: emergency stop
