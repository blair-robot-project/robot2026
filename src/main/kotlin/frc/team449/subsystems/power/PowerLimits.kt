package frc.team449.subsystems.power

// shooter and hood always get priority
data class PowerLimits(
    val driveSpeedMultiplier: Double,
    val intakeSpeedMultiplier: Double,
    val indexerSpeedMultiplier: Double
)
