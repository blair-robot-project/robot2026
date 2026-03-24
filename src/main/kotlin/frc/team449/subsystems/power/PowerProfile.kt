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
            24.0,
            6.0,
        ),
    ),
    SHOOTING(
        PowerLimits(
            6.0,
            24.0,
        ),
    )
}
