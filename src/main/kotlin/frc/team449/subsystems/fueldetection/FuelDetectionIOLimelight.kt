package frc.team449.subsystems.fueldetection

import frc.team449.Constants.FuelDetectionConstants
import frc.team449.subsystems.fueldetection.FuelDetectionIO.FuelDetection
import frc.team449.subsystems.fueldetection.FuelDetectionIO.FuelDetectionIOInputs
import limelight.Limelight
import limelight.networktables.LimelightSettings

class FuelDetectionIOLimelight(
    name: String,
    detectorPipelineIndex: Int
) : FuelDetectionIO {
    private val limelight = Limelight(name)

    init {
        limelight.settings
            .withPipelineIndex(detectorPipelineIndex)
            .withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
            .save()
    }

    override fun updateInputs(inputs: FuelDetectionIOInputs) {
        val resultsOpt = limelight.data.results

        if (resultsOpt.isPresent) {
            val results = resultsOpt.get()
            inputs.connected = true
            inputs.pipelineLatencyMs = results.latency_pipeline
            inputs.captureLatencyMs = results.latency_capture

            val detectorTargets = results.targets_Detector
            if (detectorTargets != null && detectorTargets.isNotEmpty()) {
                inputs.detections = detectorTargets
                    .filter { it.confidence >= FuelDetectionConstants.MIN_CONFIDENCE }
                    .map { target ->
                        FuelDetection(
                            tx = target.tx,
                            ty = target.ty,
                            ta = target.ta,
                            confidence = target.confidence
                        )
                    }.toTypedArray()
            } else {
                inputs.detections = emptyArray()
            }
        } else {
            inputs.connected = false
            inputs.detections = emptyArray()
            inputs.pipelineLatencyMs = 0.0
            inputs.captureLatencyMs = 0.0
        }
    }

    override fun setPipeline(index: Int) {
        limelight.settings.withPipelineIndex(index).save()
    }
}
