package frc.team449.commands

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants.ShooterConstants
import frc.team449.subsystems.shooter.ShooterSubsystem
import java.util.function.Supplier

class AimbotShooterCommand(
    private val shooter: ShooterSubsystem,
    private val distanceSupplier: Supplier<Double>
) : Command() {

    init {
        addRequirements(shooter)
    }

    override fun initialize() {
        println("Initializing AimbotShooterCommand")
    }

    override fun execute() {
        val angle = Degrees.of(ShooterConstants.HOOD_ANGLE_MAP.get(distanceSupplier.get()))
        shooter.setHoodAngle(angle)
        val velocity = RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distanceSupplier.get()))
        shooter.setFlywheelVelocity(velocity)
    }
}
