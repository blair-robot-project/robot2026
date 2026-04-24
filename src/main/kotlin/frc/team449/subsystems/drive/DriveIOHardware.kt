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
import edu.wpi.first.units.measure.Current
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

    private val frontLeftCurrentSignals = getModuleCurrentSignals(0)
    private val frontRightCurrentSignals = getModuleCurrentSignals(1)
    private val backLeftCurrentSignals = getModuleCurrentSignals(2)
    private val backRightCurrentSignals = getModuleCurrentSignals(3)

    private val driveCurrentSignals =
        arrayOf(
            frontLeftCurrentSignals,
            frontRightCurrentSignals,
            backLeftCurrentSignals,
            backRightCurrentSignals,
        )
            .flatten()
            .toTypedArray()

    init {
        this.odometryThread.setThreadPriority(99)
        registerTelemetry(telemetryConsumer)

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, *gyroSignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *driveCurrentSignals)

        ParentDevice.optimizeBusUtilizationForAll(pigeon2, *modules.flatMap { listOf(it.driveMotor, it.steerMotor) }.toTypedArray())
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        BaseStatusSignal.refreshAll(
            *gyroSignals,
            *driveCurrentSignals
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
            frontLeftCurrentSignals[0].value.`in`(Units.Amps),
            frontLeftCurrentSignals[1].value.`in`(Units.Amps),
            frontLeftCurrentSignals[2].value.`in`(Units.Amps),
            frontLeftCurrentSignals[3].value.`in`(Units.Amps)
        )
        inputs.frontRightData = ModuleData(
            frontRightCurrentSignals[0].value.`in`(Units.Amps),
            frontRightCurrentSignals[1].value.`in`(Units.Amps),
            frontRightCurrentSignals[2].value.`in`(Units.Amps),
            frontRightCurrentSignals[3].value.`in`(Units.Amps)
        )
        inputs.backLeftData = ModuleData(
            backLeftCurrentSignals[0].value.`in`(Units.Amps),
            backLeftCurrentSignals[1].value.`in`(Units.Amps),
            backLeftCurrentSignals[2].value.`in`(Units.Amps),
            backLeftCurrentSignals[3].value.`in`(Units.Amps)
        )
        inputs.backRightData = ModuleData(
            backRightCurrentSignals[0].value.`in`(Units.Amps),
            backRightCurrentSignals[1].value.`in`(Units.Amps),
            backRightCurrentSignals[2].value.`in`(Units.Amps),
            backRightCurrentSignals[3].value.`in`(Units.Amps)
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

    private fun getModuleCurrentSignals(moduleIndex: Int): Array<StatusSignal<Current>> =
        arrayOf(
            modules[moduleIndex].driveMotor.supplyCurrent,
            modules[moduleIndex].driveMotor.statorCurrent,
            modules[moduleIndex].steerMotor.supplyCurrent,
            modules[moduleIndex].steerMotor.statorCurrent,
        )
}
