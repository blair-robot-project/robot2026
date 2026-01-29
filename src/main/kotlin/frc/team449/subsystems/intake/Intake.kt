package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.intake.pivot.PivotIO
import frc.team449.subsystems.intake.pivot.PivotIOInputsAutoLogged
import frc.team449.subsystems.intake.roller.RollerIO
import frc.team449.subsystems.intake.roller.RollerIOInputsAutoLogged
import org.littletonrobotics.junction.Logger

class Intake(
    private val pivotIo: PivotIO,
    private val rollerIO: RollerIO
) : SubsystemBase() {
    private val pivotInputs: PivotIOInputsAutoLogged = PivotIOInputsAutoLogged()
    private val rollerInputs: RollerIOInputsAutoLogged = RollerIOInputsAutoLogged()

    private var targetAngle: Angle = Radians.of(0.0)
    private var targetVel: Double = 0.0

    override fun periodic() {
        pivotIo.updateInputs(pivotInputs)
        rollerIO.updateInputs(rollerInputs)

        Logger.processInputs("Intake/Pivot", pivotInputs)
        Logger.processInputs("Intake/Roller", rollerInputs)
        Logger.recordOutput("Intake/Pivot/targetAngle", targetAngle)
        Logger.recordOutput("Intake/Roller/targetVelocity", targetVel)
    }

    fun intake() {
        targetAngle = Radians.of(0.0)
        targetVel = 50.0

        pivotIo.setAngle(targetAngle)
        rollerIO.setVelocity(targetVel)
    }

    fun stow() {
        targetAngle = Degrees.of(90.0)
        targetVel = 0.0

        pivotIo.setAngle(targetAngle)
        rollerIO.setVelocity(targetVel)
    }
}
