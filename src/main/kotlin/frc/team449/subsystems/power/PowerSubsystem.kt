package frc.team449.subsystems.power

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object PowerSubsystem : SubsystemBase() {
    var lastProfile: PowerProfile = PowerProfile.LOW_BATTERY
    var currentProfile: PowerProfile = PowerProfile.DRIVING

    override fun periodic() {
        if (currentProfile == lastProfile) return
        lastProfile = currentProfile

        val currentProfileLimits = currentProfile.limits

        Logger.recordOutput(
            "Power/CurrentLimits",
            doubleArrayOf(
                currentProfileLimits.intakeSlewRate,
                currentProfileLimits.hopperSlewRate,
            )
        )
    }

    fun requestProfile(requestedProfile: PowerProfile): Command =
        runOnce {
            this.currentProfile = requestedProfile
        }
}
