package frc.team449.subsystems.power

enum class PowerProfile(
    val limits: PowerLimits
) {
    LOW_BATTERY(
        PowerLimits(
            20.0,
            5.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
    ),
    DRIVING(
        PowerLimits(
            60.0,
            15.0,
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
        ),
    ),
    INTAKING(
        PowerLimits(
            30.0,
            10.0,
            10.0,
            10.0,
            5.0,
            5.0,
            5.0,
            5.0,
        ),
    ),
    SHOOTING(
        PowerLimits(
            10.0,
            15.0,
            5.0,
            30.0,
            5.0,
            30.0,
            30.0,
            20.0,
        ),
    )
}
