package frc.team449.subsystems.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.simulation.MockLaserCan
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.subsystems.superstructure.intake.IntakeConstants.config
import frc.team449.system.motor.KrakenDogLog
import kotlin.math.abs

class Intake(
    private val topMotor: TalonFX,
    private val rightMotor: TalonFX,
    private val leftMotor: TalonFX,
    private val backSensor: LaserCanInterface,
    private val leftSensor: LaserCanInterface,
    private val rightSensor: LaserCanInterface,
    private val middleSensor: LaserCanInterface,
) : SubsystemBase() {
    private val sensors =
        listOf(
            backSensor,
            leftSensor,
            rightSensor,
            middleSensor,
        )

    private fun setVoltage(
        vararg motors: TalonFX,
        voltage: Double,
    ): Command =
        runOnce {
            motors.forEach { it.setVoltage(voltage) }
        }
}
