package frc.team449.subsystems.drive

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Notifier
import frc.team449.Constants
import frc.team449.Constants.DriveConstants.SIM_LOOP_TIME
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import java.util.function.Consumer

class DriveIOSim(
    driveConstants: SwerveDrivetrainConstants,
    moduleConstants: Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
) : DriveIOHardware(
    driveConstants,
    sanitizeConstantsForSim(moduleConstants)
) {

    val simTelemetryConsumer: Consumer<SwerveDriveState> = Consumer { swerveDriveState: SwerveDriveState ->
        swerveDriveState.Pose = mapleSimDrive.simulatedDriveTrainPose
        telemetryConsumer.accept(swerveDriveState)
    }

    val simulationConfig: DriveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
        .withBumperSize(Inches.of(Constants.ROBOT_LENGTH_INCHES), Inches.of(Constants.ROBOT_WIDTH_INCHES))
        .withGyro(COTS.ofPigeon2())
        .withTrackLengthTrackWidth(
            Inches.of(Constants.DriveConstants.TRACKWIDTH_INCHES),
            Inches.of(Constants.DriveConstants.WHEELBASE_INCHES)
        )
        .withSwerveModule(
            SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getNEO(1),
                moduleConstants[0].DriveMotorGearRatio,
                moduleConstants[0].SteerMotorGearRatio,
                Volts.of(moduleConstants[0].DriveFrictionVoltage),
                Volts.of(moduleConstants[0].SteerFrictionVoltage),
                Meters.of(moduleConstants[0].WheelRadius),
                KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                1.2
            )
        )

    val mapleSimDrive = SwerveDriveSimulation(simulationConfig, Pose2d(3.0, 3.0, Rotation2d()))

    private val simNotifier = Notifier {
        SimulatedArena.getInstance().simulationPeriodic()

        pigeon2.simState.setRawYaw(mapleSimDrive.simulatedDriveTrainPose.rotation.measure)
        pigeon2.simState.setAngularVelocityZ(
            RadiansPerSecond.of(
                mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                    .omegaRadiansPerSecond
            )
        )
    }

    init {
        initializeSimulation()

        registerTelemetry(simTelemetryConsumer)
        simNotifier.startPeriodic(SIM_LOOP_TIME)
    }

    fun initializeSimulation() {
        SimulatedArena.overrideSimulationTimings(Seconds.of(SIM_LOOP_TIME), 1)
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive)

        for (i in 0 until 4) {
            val realModule = this.getModule(i)
            val simModule = mapleSimDrive.modules[i]

            simModule.useDriveMotorController(
                SimulatedMotorController { _, _, encAngle, encVel ->
                    realModule.driveMotor.simState.setRawRotorPosition(encAngle)
                    realModule.driveMotor.simState.setRotorVelocity(encVel)
                    realModule.driveMotor.simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage())

                    realModule.driveMotor.simState.motorVoltageMeasure
                }
            )

            simModule.useSteerMotorController(
                SimulatedMotorController { mechPos, mechVel, encPos, encVel ->
                    realModule.encoder.simState.setRawPosition(mechPos)
                    realModule.encoder.simState.setVelocity(mechVel)

                    realModule.steerMotor.simState.setRawRotorPosition(encPos)
                    realModule.steerMotor.simState.setRotorVelocity(encVel)
                    realModule.steerMotor.simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage())

                    realModule.steerMotor.simState.motorVoltageMeasure
                }
            )
        }
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        super.updateInputs(inputs)
    }

    override fun resetOdometry(pose: Pose2d) {
        mapleSimDrive.setSimulationWorldPose(pose)
        super.resetPose(pose)
    }

    companion object {
        private fun sanitizeConstantsForSim(
            originalConstants: Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
        ): Array<SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> {
            // create a new array to hold the modified constants
            return originalConstants.map { module ->
                // create a modified copy of the module constant
                module
                    .withEncoderOffset(0.0)
                    .withDriveMotorInverted(false)
                    .withSteerMotorInverted(false)
                    .withEncoderInverted(false)
                    .withSteerMotorGains(
                        module
                            .SteerMotorGains
                            .withKP(70.0)
                            .withKD(4.5)
                    )
                    .withDriveFrictionVoltage(Volts.of(0.1))
                    .withSteerFrictionVoltage(Volts.of(0.15))
                    .withSteerInertia(KilogramSquareMeters.of(0.05))
            }.toTypedArray()
        }
    }
}
