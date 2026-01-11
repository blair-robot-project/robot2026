package frc.team449.subsystems.drive

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.AutoLog

interface DriveIO {
    @AutoLog
    open class DriveIOInputs : SwerveDriveState() {
        @JvmField var gyroAngle: Double = 0.0

        init {
            this.Pose = Pose2d()
        }

        fun fromSwerveDriveState(stateIn: SwerveDriveState) {
            this.Pose = stateIn.Pose
            this.SuccessfulDaqs = stateIn.SuccessfulDaqs
            this.FailedDaqs = stateIn.FailedDaqs
            this.ModuleStates = stateIn.ModuleStates
            this.ModuleTargets = stateIn.ModuleTargets
            this.Speeds = stateIn.Speeds
            this.OdometryPeriod = stateIn.OdometryPeriod
        }
    }

    fun updateInputs(inputs: DriveIOInputs) {}

    fun logModules(driveState: SwerveDriveState) {}

    fun resetOdometry(pose: Pose2d) {}

    fun setControl(request: SwerveRequest) {}

    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) {}

    fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {}
}
