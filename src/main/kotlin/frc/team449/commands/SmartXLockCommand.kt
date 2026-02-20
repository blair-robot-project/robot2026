package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.subsystems.drive.DriveSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.abs

class SmartXLockCommand(
    private val drive: DriveSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val turnSupplier: DoubleSupplier
) : Command() {
    private val xLockRequest = SwerveRequest.SwerveDriveBrake()
    private val xLockDeadband = Constants.DriveConstants.X_LOCK_DEADBAND

    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.setControl(xLockRequest)
    }

    override fun isFinished(): Boolean {
        return abs(throttleSupplier.asDouble) > xLockDeadband ||
            abs(strafeSupplier.asDouble) > xLockDeadband ||
            abs(turnSupplier.asDouble) > xLockDeadband
    }
}
