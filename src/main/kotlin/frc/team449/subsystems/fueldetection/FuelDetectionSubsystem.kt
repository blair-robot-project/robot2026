package frc.team449.subsystems.fueldetection

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.FuelDetectionConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.tan

class FuelDetectionSubsystem(
    private val io: FuelDetectionIO
) : SubsystemBase() {
    private val inputs = FuelDetectionIOInputsAutoLogged()
    private val disconnectedAlert = Alert("Fuel Detection Camera Disconnected.", AlertType.kWarning)

    val hasFuel: Boolean
        get() = inputs.detections.isNotEmpty()

    val bestDetectionTx: Rotation2d
        get() = if (inputs.detections.isNotEmpty()) {
            Rotation2d.fromDegrees(inputs.detections[0].tx)
        } else {
            Rotation2d.kZero
        }

    val bestDetectionTy: Rotation2d
        get() = if (inputs.detections.isNotEmpty()) {
            Rotation2d.fromDegrees(inputs.detections[0].ty)
        } else {
            Rotation2d.kZero
        }

    /**
     * Pinhole-model distance estimate to the best detected fuel.
     *
     * NeuralDetector ty is positive-down from the crosshair. The camera pitch
     * (positive = above horizontal) is subtracted so that
     * `angleBelowHorizontal = ty_rad − cameraPitch`.
     *
     * Returns [Double.MAX_VALUE] when no detection exists or the geometry
     * is degenerate (target at or above horizontal).
     */
    val estimatedDistanceMeters: Double
        get() {
            if (inputs.detections.isEmpty()) return Double.MAX_VALUE
            val tyRad = Units.degreesToRadians(inputs.detections[0].ty)
            val angleBelowHorizontal = tyRad - FuelDetectionConstants.CAMERA_PITCH_RAD
            if (angleBelowHorizontal <= 0.01) return Double.MAX_VALUE
            val heightDelta = FuelDetectionConstants.CAMERA_HEIGHT_METERS - FuelDetectionConstants.FUEL_CENTER_HEIGHT_METERS
            return abs(heightDelta / tan(angleBelowHorizontal))
        }

    val bestConfidence: Double
        get() = if (inputs.detections.isNotEmpty()) inputs.detections[0].confidence else 0.0

    val detectionCount: Int
        get() = inputs.detections.size

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("FuelDetection", inputs)

        disconnectedAlert.set(!inputs.connected)

        Logger.recordOutput("FuelDetection/HasFuel", hasFuel)
        Logger.recordOutput("FuelDetection/DetectionCount", detectionCount)

        if (hasFuel) {
            Logger.recordOutput("FuelDetection/BestTxDeg", inputs.detections[0].tx)
            Logger.recordOutput("FuelDetection/BestTyDeg", inputs.detections[0].ty)
            Logger.recordOutput("FuelDetection/BestArea", inputs.detections[0].ta)
            Logger.recordOutput("FuelDetection/BestConfidence", bestConfidence)
            Logger.recordOutput("FuelDetection/EstDistMeters", estimatedDistanceMeters)
        }
    }
}
