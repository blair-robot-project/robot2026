package frc.team449.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.TalonFXS
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
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
    100.0,
    *moduleConstants
),
    DriveIO {

    var telemetryCache: AtomicReference<SwerveDriveState> = AtomicReference()

    var telemetryConsumer: Consumer<SwerveDriveState> = Consumer {
            swerveDriveState: SwerveDriveState ->
        telemetryCache.set(swerveDriveState.clone())
    }

    val angularPitchVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityYWorld
    val angularRollVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityXWorld
    val angularYawVelocity: StatusSignal<AngularVelocity> = pigeon2.angularVelocityZWorld
    val roll: StatusSignal<Angle> = pigeon2.roll
    val pitch: StatusSignal<Angle> = pigeon2.pitch
    val accelX: StatusSignal<LinearAcceleration> = pigeon2.accelerationX
    val accelY: StatusSignal<LinearAcceleration> = pigeon2.accelerationY

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            angularPitchVelocity,
            angularRollVelocity,
            angularYawVelocity,
            roll,
            pitch,
            accelX,
            accelY
        )

        pigeon2.optimizeBusUtilization()

        this.odometryThread.setThreadPriority(99)
        registerTelemetry(telemetryConsumer)
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        if (telemetryCache.get() == null) return
        inputs.fromSwerveDriveState(telemetryCache.get())

        inputs.gyroAngle = inputs.Pose.rotation.degrees

        BaseStatusSignal.refreshAll(
            angularRollVelocity,
            angularPitchVelocity,
            angularYawVelocity,
            pitch,
            roll,
            accelX,
            accelY
        )
    }

    override fun resetOdometry(pose: Pose2d) {
        super.resetPose(pose)
    }

    override fun setControl(request: SwerveRequest) {
        super<SwerveDrivetrain>.setControl(request)
    }

    override fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>
    ) {
        super<SwerveDrivetrain>.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    override fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        this.setStateStdDevs(visionMeasurementStdDevs)
    }

    override fun logModules(driveState: SwerveDriveState) {
        val moduleNames = arrayOf("Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR")
        if (driveState.ModuleStates == null || driveState.ModulePositions == null) return
        for (i in 0 until modules.count()) {
            Logger.recordOutput(
                moduleNames[i] + "/Steering Angle",
                driveState.ModuleStates[i].angle
            )
            Logger.recordOutput(
                moduleNames[i] + "/Target Steering Angle",
                driveState.ModuleTargets[i].angle
            )
            Logger.recordOutput(
                moduleNames[i] + "/Drive Velocity",
                driveState.ModuleStates[i].speedMetersPerSecond
            )
            Logger.recordOutput(
                moduleNames[i] + "/Target Drive Velocity",
                driveState.ModuleTargets[i].speedMetersPerSecond
            )
            Logger.recordOutput(
                moduleNames[i] + "/Drive Position",
                driveState.ModulePositions[i].distanceMeters
            )
        }
    }
}
