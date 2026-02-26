package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.drive.DriveSubsystem

class XLockCommand(
    private val drive: DriveSubsystem
) : Command() {
    private val xLockRequest = SwerveRequest.SwerveDriveBrake()

    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.setControl(xLockRequest)
    }
}
