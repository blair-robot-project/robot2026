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
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.Constants.DriveConstants.MODULE_ALIGN_TOLERANCE
import limelight.networktables.AngularVelocity3d
import org.littletonrobotics.junction.Logger
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

    val modulePositions: Array<SwerveModulePosition>
        get() = inputs.ModulePositions

    override fun periodic() {
        io.updateInputs(inputs)
        io.logModules(inputs)
        field.robotPose = pose

        Logger.processInputs("Drive", inputs)
        Logger.recordOutput(
            "Drive/ActiveCommand",
            currentCommand?.name ?: "None",
        )
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

    // call in autoInit() and teleopInit()
    fun setOperatorPerspectiveForward() {
        var forward: Rotation2d = Rotation2d.kZero

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            forward = Rotation2d.k180deg
        }

        io.setOperatorPerspectiveForward(forward)
        CommandScheduler.getInstance().schedule(PrintCommand(forward.toString()))
    }

    fun getRobotRelativeSpeeds(): ChassisSpeeds = inputs.Speeds

    fun getFieldRelativeSpeeds(): ChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            inputs.Speeds,
            inputs.Pose.rotation,
        )

    fun getAngularVelocity(): AngularVelocity3d =
        AngularVelocity3d(
            DegreesPerSecond.of(inputs.rollVelocityDegreesPerSecond),
            DegreesPerSecond.of(inputs.pitchVelocityDegreesPerSecond),
            DegreesPerSecond.of(inputs.yawVelocityDegreesPerSecond)
        )

    fun xLock(): Command = run { io.setControl(SwerveRequest.SwerveDriveBrake()) }

    fun alignModules(direction: Rotation2d): Command =
        run {
            io.setControl(SwerveRequest.PointWheelsAt().withModuleDirection(direction))
        }.until {
            (0..3).all { i ->
                val target = inputs.ModuleTargets[i].angle
                val state = inputs.ModuleStates[i].angle

                abs(target.minus(state).degrees) <= MODULE_ALIGN_TOLERANCE
            }
        }

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

    fun setSupplyLimits(driveSupplyLimitAmps: Double, steerSupplyLimitAmps: Double) {
        io.setSupplyLimits(driveSupplyLimitAmps, steerSupplyLimitAmps)
    }

    private val translationCharacterizationRequest = SwerveRequest.SysIdSwerveTranslation()
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
