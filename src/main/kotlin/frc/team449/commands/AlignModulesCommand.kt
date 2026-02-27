package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.drive.DriveSubsystem

class AlignModulesCommand(
    private val drive: DriveSubsystem,
    private val direction: Rotation2d
) : Command() {
    private val moduleAlignRequest = SwerveRequest.PointWheelsAt()

    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.setControl(moduleAlignRequest.withModuleDirection(direction))
    }

    override fun isFinished(): Boolean = drive.pose.rotation == direction
}
