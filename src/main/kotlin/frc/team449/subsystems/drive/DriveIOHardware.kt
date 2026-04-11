package frc.team449.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.TalonFXS
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import frc.team449.Constants
import frc.team449.util.PhoenixUtil
import org.littletonrobotics.junction.Logger
import java.util.concurrent.atomic.AtomicReference
import java.util.function.Consumer

open class DriveIOHardware(
    driveConstants: SwerveDrivetrainConstants,
    moduleConstants: Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration>>
) : SwerveDrivetrain<TalonFX, TalonFXS, CANcoder>(
    ::TalonFX,
    ::TalonFXS,
    ::CANcoder,
    driveConstants,
    Constants.DriveConstants.ODOMETRY_LOOP_HZ,
    *moduleConstants,
),
    DriveIO {
    var telemetryCache: AtomicReference<SwerveDriveState> = AtomicReference()

    var telemetryConsumer: Consumer<SwerveDriveState> =
        Consumer { swerveDriveState: SwerveDriveState ->
            telemetryCache.set(swerveDriveState.clone())
        }

    val angularPitchVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityYWorld
    val angularRollVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityXWorld
    val angularYawVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityZWorld

    val gyroSignals =
        arrayOf(
            angularPitchVelocity,
            angularRollVelocity,
            angularYawVelocity,
        )

    init {
        ParentDevice.optimizeBusUtilizationForAll(pigeon2)
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, angularYawVelocity)

        PhoenixUtil.registerSignals(*gyroSignals)

        this.odometryThread.setThreadPriority(99)

        registerTelemetry(telemetryConsumer)
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        BaseStatusSignal.refreshAll(*gyroSignals)

        if (telemetryCache.get() == null) return
        inputs.fromSwerveDriveState(telemetryCache.get())

        inputs.gyroAngle = inputs.Pose.rotation.degrees
        inputs.rollVelocityDegreesPerSecond = angularRollVelocity.value.`in`(DegreesPerSecond)
        inputs.pitchVelocityDegreesPerSecond = angularPitchVelocity.value.`in`(DegreesPerSecond)
        inputs.yawVelocityDegreesPerSecond = angularYawVelocity.value.`in`(DegreesPerSecond)
    }

    override fun setControl(request: SwerveRequest) {
        super<SwerveDrivetrain>.setControl(request)
    }

    override fun seedFieldCentric() {
        super<SwerveDrivetrain>.seedFieldCentric()
    }

    override fun resetOdometry(pose: Pose2d) {
        super.resetPose(pose)
    }

    override fun setOperatorPerspectiveForward(yaw: Rotation2d) {
        super<SwerveDrivetrain>.setOperatorPerspectiveForward(yaw)
    }

    override fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>
    ) {
        super<SwerveDrivetrain>.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    override fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        super<SwerveDrivetrain>.setStateStdDevs(visionMeasurementStdDevs)
    }

    override fun logModules(driveState: SwerveDriveState) {
        val moduleNames = arrayOf("Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR")
        if (driveState.ModuleStates == null) return
        for (i in 0 until modules.count()) {
            Logger.recordOutput(
                moduleNames[i] + "/DriveSupplyCurrentAmps",
                this.modules[i].driveMotor.supplyCurrent.valueAsDouble,
            )
            Logger.recordOutput(
                moduleNames[i] + "/DriveStatorCurrentAmps",
                this.modules[i].driveMotor.statorCurrent.valueAsDouble,
            )
            Logger.recordOutput(
                moduleNames[i] + "/SteerSupplyCurrentAmps",
                this.modules[i].steerMotor.supplyCurrent.valueAsDouble,
            )
            Logger.recordOutput(
                moduleNames[i] + "/SteerStatorCurrentAmps",
                this.modules[i].steerMotor.statorCurrent.valueAsDouble,
            )
        }
    }
}
