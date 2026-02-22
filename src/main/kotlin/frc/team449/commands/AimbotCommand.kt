package frc.team449.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem

class AimbotCommand(
    drive: DriveSubsystem,
    shooter: ShooterSubsystem
) : Command() {
    init {
        addRequirements(drive, shooter)
    }
}
