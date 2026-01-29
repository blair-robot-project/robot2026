package frc.team449.subsystems.intake.pivot

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim

class PivotIOSim : PivotIO {
    private val pivotSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            1.0, // TODO: find out
            0.0242102742,
            0.35,
            0.0,
            (130 * (Math.PI / 180)),
            true,
            0.0,
        )
    private val controller =
        PIDController(
            0.1,
            0.0,
            0.01,
        )

    private val feedforward =
        ArmFeedforward(
            0.08,
            0.10,
            4.0,
            0.0,
        )

    private var targetAngle: Angle = Radians.of(0.0)

    override fun updateInputs(pivotInputs: PivotIO.PivotIOInputs) {
        val pidVolt =
            controller.calculate(
                pivotInputs.currentAngle,
                targetAngle.`in`(Degrees),
            )
//        val ffVolt =
//            feedforward.calculate(
//                targetAngle.`in`(Degrees),
//                0.0,
//            )
//
//      //  val appliedVolt = (pidVolt + 0.0).coerceIn(-12.0, 12.0)

        pivotSim.setInputVoltage(pidVolt)
        pivotSim.update(0.02)

        pivotInputs.currentAngle = (pivotSim.angleRads) * (180.0 / Math.PI)
        pivotInputs.voltage = pidVolt
        pivotInputs.supplyCurrent = pivotSim.currentDrawAmps
    }

    override fun setAngle(angle: Angle) {
        targetAngle = angle
    }
}
