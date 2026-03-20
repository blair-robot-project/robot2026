package frc.team449.subsystems.power

enum class PowerProfile(
    val limits: PowerLimits
) {
    LOW_BATTERY(
        PowerLimits(
            6.0,
            6.0,
        ),
    ),
    DRIVING(
        PowerLimits(
            6.0,
            6.0,
        ),
    ),
    INTAKING(
        PowerLimits(
            36.0,
            12.0,
        ),
    ),
    SHOOTING(
        PowerLimits(
            6.0,
            36.0,
        ),
    )
}
