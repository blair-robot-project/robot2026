package frc.team449.subsystems.power

enum class PowerProfile(
    val limits: PowerLimits
) {
    LOW_BATTERY(PowerLimits(0.5, 0.1, 0.1)),
    DRIVING(PowerLimits(1.0, 0.5, 0.2)),
    INTAKING(PowerLimits(0.7, 1.0, 0.2)),
    SHOOTING(PowerLimits(0.4, 0.5, 1.0))
}
