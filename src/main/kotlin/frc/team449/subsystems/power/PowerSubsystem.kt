package frc.team449.subsystems.power

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import org.littletonrobotics.junction.Logger

class PowerSubsystem(
    private val drive: DriveSubsystem,
    private val intake: IntakeSubsystem,
    private val indexer: IndexerSubsystem,
    private val shooter: ShooterSubsystem
) : SubsystemBase() {
    var lastProfile: PowerProfile = PowerProfile.LOW_BATTERY
    var currentProfile: PowerProfile = PowerProfile.DRIVING

    override fun periodic() {
        if (currentProfile == lastProfile) return
        lastProfile = currentProfile

        val currentProfileLimits = currentProfile.limits
        drive.setSupplyLimits(currentProfileLimits.driveSupplyLimit, 20.0) // 20.0 steer supply limit
        intake.setSupplyLimits(currentProfileLimits.pivotSupplyLimit, currentProfileLimits.rollerSupplyLimit)
        indexer.setSupplyLimits(
            currentProfileLimits.floorIndexerSupplyLimit,
            currentProfileLimits.wedgeIndexerSupplyLimit,
            currentProfileLimits.topIndexerSupplyLimit,
        )
        shooter.setSupplyLimits(currentProfileLimits.flywheelSupplyLimit, currentProfileLimits.hoodSupplyLimit)

        Logger.recordOutput(
            "Power/CurrentLimits",
            doubleArrayOf(
                currentProfileLimits.driveSupplyLimit,
                currentProfileLimits.pivotSupplyLimit,
                currentProfileLimits.rollerSupplyLimit,
                currentProfileLimits.floorIndexerSupplyLimit,
                currentProfileLimits.wedgeIndexerSupplyLimit,
                currentProfileLimits.topIndexerSupplyLimit,
                currentProfileLimits.flywheelSupplyLimit,
                currentProfileLimits.hoodSupplyLimit,
            )
        )
    }

    fun requestProfile(requestedProfile: PowerProfile): Command =
        runOnce {
            this.currentProfile = requestedProfile
        }
}
