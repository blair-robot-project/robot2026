package frc.team449.subsystems.power

import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase

class PowerSubsystem(
    private val drive: DriveSubsystem,
    private val intake: IntakeSubsystem,
    private val indexer: IndexerSubsystem,
    private val shooter: ShooterSubsystem
) : SubsystemBase() {
    var lastProfile: PowerProfile = PowerProfile.DRIVING
    var currentProfile: PowerProfile = PowerProfile.DRIVING

    override fun periodic() {
        if (currentProfile == lastProfile) return

        val currentProfileLimits = currentProfile.limits
        drive.setSupplyLimits(currentProfileLimits.driveSupplyLimit, 20.0) // 20.0 steer supply limit
        intake.setSupplyLimits(currentProfileLimits.pivotSupplyLimit, currentProfileLimits.rollerSupplyLimit)
        indexer.setSupplyLimits(
            currentProfileLimits.floorIndexerSupplyLimit, 
            currentProfileLimits.wedgeIndexerSupplyLimit, 
            currentProfileLimits.topIndexerSupplyLimit,
        )
        shooter.setSupplyLimits(currentProfileLimits.flywheelSupplyLimit, currentProfileLimits.hoodSupplyLimit)
    } 

    fun requestProfile(requestedProfile: PowerProfile) {
        this.currentProfile = requestedProfile
    }
}

data class PowerLimits(
    val driveSupplyLimit: Double,
    val pivotSupplyLimit: Double,
    val rollerSupplyLimit: Double,
    val floorIndexerSupplyLimit: Double,
    val wedgeIndexerSupplyLimit: Double,
    val topIndexerSupplyLimit: Double,
    val flywheelSupplyLimit: Double,
    val hoodSupplyLimit: Double
)

enum class PowerProfile(
    val limits: PowerLimits
) {
    LOW_BATTERY(PowerLimits(20.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    DRIVING(PowerLimits(60.0, 15.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0)),
    INTAKING(PowerLimits(30.0, 10.0, 40.0, 10.0, 5.0, 5.0, 0.0, 0.0)),
    SHOOTING(PowerLimits(10.0, 15.0, 5.0, 10.0, 10.0, 30.0, 30.0, 20.0)) 
}
