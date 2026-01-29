package frc.team449.subsystems.intake

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.intake.pivot.PivotIO
import frc.team449.subsystems.intake.pivot.PivotIOInputsAutoLogged
import frc.team449.subsystems.intake.roller.RollerIO
import frc.team449.subsystems.intake.roller.RollerIOInputsAutoLogged
import org.littletonrobotics.junction.Logger

class Intake(
    private val pivotIo: PivotIO,
    private val rollerIO: RollerIO,
) : SubsystemBase() {
    private val pivotInputs: PivotIOInputsAutoLogged = PivotIOInputsAutoLogged()
    private val rollerInputs: RollerIOInputsAutoLogged = RollerIOInputsAutoLogged()

    override fun periodic() {
        pivotIo.updateInputs(pivotInputs)
        rollerIO.updateInputs(rollerInputs)

        Logger.processInputs("Intake/Pivot", pivotInputs)
        Logger.processInputs("Intake/Roller", rollerInputs)
    }

    fun intake() {}

    fun stow() {}
}
