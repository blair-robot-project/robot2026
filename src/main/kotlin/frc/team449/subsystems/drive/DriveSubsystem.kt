package frc.team449.subsystems.drive

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.Constants.DriveConstants
import frc.team449.util.FieldUtil
import limelight.networktables.AngularVelocity3d
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrElse
import kotlin.math.abs

class DriveSubsystem(
    private val io: DriveIO
) : SubsystemBase() {
    private val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()
    private val field: Field2d = Field2d().apply {
        SmartDashboard.putData("Field", this)
    }

    val pose: Pose2d
        get() = inputs.Pose

    val robotRelativeSpeeds: ChassisSpeeds
        get() = inputs.Speeds

    val fieldRelativeSpeeds: ChassisSpeeds
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(
            inputs.Speeds,
            inputs.Pose.rotation,
        )

    val angularVelocity: AngularVelocity3d
        get() = AngularVelocity3d(
            DegreesPerSecond.of(inputs.rollVelocityDegreesPerSecond),
            DegreesPerSecond.of(inputs.pitchVelocityDegreesPerSecond),
            DegreesPerSecond.of(inputs.yawVelocityDegreesPerSecond)
        )

    val modulePositions: Array<SwerveModulePosition>
        get() = inputs.ModulePositions

    private val translationCharacterizationRequest = SwerveRequest.SysIdSwerveTranslation()
    private val brakeRequest = SwerveRequest.SwerveDriveBrake()
    private val alignModulesRequest = SwerveRequest.PointWheelsAt()

    override fun periodic() {
        io.updateInputs(inputs)
        io.logModules(inputs)
        field.robotPose = pose

        Logger.processInputs("Drive", inputs)
        Logger.recordOutput("Drive/ActiveCommand", currentCommand?.name ?: "None")
        Logger.recordOutput("Drive/DistanceToHub", FieldUtil.getDistanceToFriendlyHub(pose.translation))
    }

    fun setControl(request: SwerveRequest) {
        io.setControl(request)
    }

    fun resetOdometry(pose: Pose2d) {
        io.resetOdometry(pose)
    }

    fun seedFieldCentric(): Command = runOnce {
        io.seedFieldCentric()
    }

    fun setOperatorPerspectiveForward() {
        val forward: Rotation2d =
            if (DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red) {
                Rotation2d.k180deg
            } else {
                Rotation2d.kZero
            }

        io.setOperatorPerspectiveForward(forward)
    }

    fun xLock(): Command =
        run {
            io.setControl(brakeRequest)
        }
            .withName("X-LOCK")

    fun alignModules(direction: Rotation2d): Command =
        run {
            io.setControl(alignModulesRequest.withModuleDirection(direction))
        }.until {
            (0..3).all { i ->
                val target = inputs.ModuleTargets[i].angle
                val state = inputs.ModuleStates[i].angle

                abs(target.minus(state).degrees) <= DriveConstants.MODULE_ALIGN_TOLERANCE_DEG
            }
        }
            .withName("ALIGN")
            .withTimeout(0.5)

    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>
    ) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        io.setStateStdDevs(visionMeasurementStdDevs)
    }

    val sysIDTranslationRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // default ramp rate (1 V/s)
                Volts.of(6.0), // dynamic step voltage
                null, // default timeout (10 s)
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdTranslation_State",
                    state.toString(),
                )
            },
            Mechanism(
                { output: Voltage -> setControl(translationCharacterizationRequest.withVolts(output)) },
                null,
                this,
            ),
        )
}
