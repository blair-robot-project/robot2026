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
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import frc.team449.Constants
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
    private var telemetryCache: AtomicReference<SwerveDriveState> = AtomicReference()

    var telemetryConsumer: Consumer<SwerveDriveState> =
        Consumer { swerveDriveState: SwerveDriveState ->
            telemetryCache.set(swerveDriveState.clone())
        }

    private val angularPitchVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityYWorld
    private val angularRollVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityXWorld
    private val angularYawVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityZWorld

    private val gyroSignals =
        arrayOf(
            angularPitchVelocity,
            angularRollVelocity,
            angularYawVelocity,
        )

    private val frontLeftSignals = getModuleSignals(0)
    private val frontRightSignals = getModuleSignals(1)
    private val backLeftSignals = getModuleSignals(2)
    private val backRightSignals = getModuleSignals(3)

    private val moduleSignals =
        arrayOf(
            frontLeftSignals,
            frontRightSignals,
            backLeftSignals,
            backRightSignals,
        )
            .flatten()
            .toTypedArray()

    init {
        this.odometryThread.setThreadPriority(99)
        registerTelemetry(telemetryConsumer)

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, *gyroSignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *moduleSignals)

        ParentDevice.optimizeBusUtilizationForAll(pigeon2, *modules.flatMap { listOf(it.driveMotor, it.steerMotor) }.toTypedArray())
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        BaseStatusSignal.refreshAll(
            *gyroSignals,
            *moduleSignals,
        )

        val cachedTelemetry = telemetryCache.get()
        if (cachedTelemetry != null) {
            inputs.fromSwerveDriveState(cachedTelemetry)
            inputs.gyroAngle = inputs.Pose.rotation.degrees
        }

        inputs.rollVelocityDegreesPerSecond = angularRollVelocity.value.`in`(Units.DegreesPerSecond)
        inputs.pitchVelocityDegreesPerSecond = angularPitchVelocity.value.`in`(Units.DegreesPerSecond)
        inputs.yawVelocityDegreesPerSecond = angularYawVelocity.value.`in`(Units.DegreesPerSecond)

        inputs.frontLeftData = ModuleData(
            frontLeftSignals[0].valueAsDouble,
            frontLeftSignals[1].valueAsDouble,
            frontLeftSignals[2].valueAsDouble,
            frontLeftSignals[3].valueAsDouble,
            frontLeftSignals[4].valueAsDouble,
            frontLeftSignals[5].valueAsDouble,
        )
        inputs.frontRightData = ModuleData(
            frontRightSignals[0].valueAsDouble,
            frontRightSignals[1].valueAsDouble,
            frontRightSignals[2].valueAsDouble,
            frontRightSignals[3].valueAsDouble,
            frontRightSignals[4].valueAsDouble,
            frontRightSignals[5].valueAsDouble,
        )
        inputs.backLeftData = ModuleData(
            backLeftSignals[0].valueAsDouble,
            backLeftSignals[1].valueAsDouble,
            backLeftSignals[2].valueAsDouble,
            backLeftSignals[3].valueAsDouble,
            backLeftSignals[4].valueAsDouble,
            backLeftSignals[5].valueAsDouble,
        )
        inputs.backRightData = ModuleData(
            backRightSignals[0].valueAsDouble,
            backRightSignals[1].valueAsDouble,
            backRightSignals[2].valueAsDouble,
            backRightSignals[3].valueAsDouble,
            backRightSignals[4].valueAsDouble,
            backRightSignals[5].valueAsDouble,
        )
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
        if (driveState.ModuleStates == null) return

        // add specific logging here
    }

    private fun getModuleSignals(moduleIndex: Int): Array<BaseStatusSignal> =
        arrayOf(
            modules[moduleIndex].driveMotor.motorVoltage,
            modules[moduleIndex].driveMotor.supplyCurrent,
            modules[moduleIndex].driveMotor.statorCurrent,
            modules[moduleIndex].steerMotor.motorVoltage,
            modules[moduleIndex].steerMotor.supplyCurrent,
            modules[moduleIndex].steerMotor.statorCurrent,
        )
}
