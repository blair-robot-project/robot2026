package frc.team449.subsystems.power

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
