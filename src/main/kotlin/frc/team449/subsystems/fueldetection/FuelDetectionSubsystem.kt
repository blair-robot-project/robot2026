package frc.team449.subsystems.fueldetection

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.FuelDetectionConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan

class FuelDetectionSubsystem(
    private val io: FuelDetectionIO
) : SubsystemBase() {
    private val inputs = FuelDetectionIOInputsAutoLogged()
    private val disconnectedAlert = Alert("Fuel Detection Camera Disconnected.", AlertType.kWarning)

    data class FuelClump(
        val count: Int,
        val avgTxDeg: Double,
        val avgTyDeg: Double,
        val avgDistanceMeters: Double,
        val densityScore: Double,
        val intakeAlignmentErrorDeg: Double
    )

    private var clumpsInternal: List<FuelClump> = emptyList()
    val clumps: List<FuelClump>
        get() = clumpsInternal

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
            return estimateDistanceMetersForTy(inputs.detections[0].ty)
        }

    val bestConfidence: Double
        get() = if (inputs.detections.isNotEmpty()) inputs.detections[0].confidence else 0.0

    val detectionCount: Int
        get() = inputs.detections.size

    private fun estimateDistanceMetersForTy(tyDeg: Double): Double {
        val tyRad = Units.degreesToRadians(tyDeg)
        val angleBelowHorizontal = tyRad - FuelDetectionConstants.CAMERA_PITCH_RAD
        if (angleBelowHorizontal <= 0.01) return Double.MAX_VALUE
        val heightDelta =
            FuelDetectionConstants.CAMERA_HEIGHT_METERS - FuelDetectionConstants.FUEL_CENTER_HEIGHT_METERS
        return abs(heightDelta / tan(angleBelowHorizontal))
    }

    private fun computeClumps() {
        val detections = inputs.detections
        if (detections.isEmpty()) {
            clumpsInternal = emptyList()
            return
        }

        // Attach a distance estimate to each detection and discard ones with invalid geometry.
        val withDistance =
            detections.mapNotNull { det ->
                val dist = estimateDistanceMetersForTy(det.ty)
                if (dist == Double.MAX_VALUE) null else Triple(det, dist, false)
            }

        if (withDistance.isEmpty()) {
            clumpsInternal = emptyList()
            return
        }

        val mutable = withDistance.toMutableList()
        val result = mutableListOf<FuelClump>()

        while (mutable.isNotEmpty()) {
            val seed = mutable.removeAt(0)
            val cluster = mutableListOf(seed)

            var i = 0
            while (i < mutable.size) {
                val cand = mutable[i]
                val txDiff = abs(cand.first.tx - seed.first.tx)
                val tyDiff = abs(cand.first.ty - seed.first.ty)
                val distDiff = abs(cand.second - seed.second)

                if (txDiff <= FuelDetectionConstants.CLUSTER_MAX_TX_DEG &&
                    tyDiff <= FuelDetectionConstants.CLUSTER_MAX_TY_DEG &&
                    distDiff <= FuelDetectionConstants.CLUSTER_MAX_DISTANCE_METERS
                ) {
                    cluster.add(cand)
                    mutable.removeAt(i)
                } else {
                    i++
                }
            }

            val count = cluster.size
            val avgTx = cluster.map { it.first.tx }.average()
            val avgTy = cluster.map { it.first.ty }.average()
            val avgDist = cluster.map { it.second }.average()

            var minTx = Double.POSITIVE_INFINITY
            var maxTx = Double.NEGATIVE_INFINITY
            var minTy = Double.POSITIVE_INFINITY
            var maxTy = Double.NEGATIVE_INFINITY

            for (c in cluster) {
                minTx = min(minTx, c.first.tx)
                maxTx = max(maxTx, c.first.tx)
                minTy = min(minTy, c.first.ty)
                maxTy = max(maxTy, c.first.ty)
            }

            val spreadTx = maxTx - minTx
            val spreadTy = maxTy - minTy

            // Higher when we have more detections, packed into a tighter angular region, and closer.
            val densityScore =
                count / ((spreadTx + 1.0) * (spreadTy + 1.0) * (avgDist + 1.0))

            val intakeAlignError =
                abs(avgTx - FuelDetectionConstants.INTAKE_FORWARD_ANGLE_DEG)

            result.add(
                FuelClump(
                    count = count,
                    avgTxDeg = avgTx,
                    avgTyDeg = avgTy,
                    avgDistanceMeters = avgDist,
                    densityScore = densityScore,
                    intakeAlignmentErrorDeg = intakeAlignError
                )
            )
        }

        // Sort by "goodness": highest density first, then closest, then best intake alignment.
        clumpsInternal =
            result.sortedWith(
                compareByDescending<FuelClump> { it.densityScore }
                    .thenBy { it.avgDistanceMeters }
                    .thenBy { it.intakeAlignmentErrorDeg }
            )
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("FuelDetection", inputs)
        computeClumps()

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

        // Log clump-level information for use in auto routines and post-match analysis.
        Logger.recordOutput("FuelDetection/Clumps/Count", clumps.size)
        clumps.take(3).forEachIndexed { index, clump ->
            val base = "FuelDetection/Clumps/$index"
            Logger.recordOutput("$base/Detections", clump.count)
            Logger.recordOutput("$base/AvgTxDeg", clump.avgTxDeg)
            Logger.recordOutput("$base/AvgTyDeg", clump.avgTyDeg)
            Logger.recordOutput("$base/AvgDistMeters", clump.avgDistanceMeters)
            Logger.recordOutput("$base/DensityScore", clump.densityScore)
            Logger.recordOutput("$base/IntakeAlignErrorDeg", clump.intakeAlignmentErrorDeg)
        }
    }
}
