package frc.team449

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.FuelSim
import frc.team449.subsystems.Intake
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.Indexer
import frc.team449.subsystems.indexer.IndexerIO
import frc.team449.subsystems.indexer.IndexerIOHardware
import frc.team449.subsystems.indexer.IndexerIOSim
import frc.team449.subsystems.intake.Intake
import frc.team449.subsystems.intake.IntakeIO
import frc.team449.subsystems.intake.IntakeIOHardware
import frc.team449.subsystems.intake.IntakeIOSim
import frc.team449.subsystems.shooter.ShooterIO
import frc.team449.subsystems.shooter.ShooterIOHardware
import frc.team449.subsystems.shooter.ShooterIOSim
import frc.team449.subsystems.shooter.ShooterSubsystem

object RobotContainer {
    // driver/op controllers
    val driveController: CommandXboxController = CommandXboxController(0)
    val opController: CommandXboxController = CommandXboxController(1)

    val autonomousCommand = PrintCommand("This is the autonomous command!")

    val drive: DriveSubsystem =
        DriveSubsystem(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> {
                    DriveIOHardware(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight,
                        ),
                    )
                }

                Mode.SIM -> {
                    DriveIOSim(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight,
                        ),
                    )
                }

                Mode.REPLAY -> {
                    object : DriveIO {}
                }
            },
        )

    val intake: Intake =
        Intake(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> IntakeIOHardware()
                Mode.SIM -> IntakeIOSim()
                Mode.REPLAY -> object : IntakeIO {}
            },
        )

    val indexer: Indexer =
        Indexer(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> IndexerIOHardware()
                Mode.SIM -> IndexerIOSim()
                Mode.REPLAY -> object : IndexerIO {}
            },
        )
    val shooter: ShooterSubsystem = ShooterSubsystem(
        when (Constants.CURRENT_MODE) {
            Mode.REAL -> ShooterIOHardware()
            Mode.SIM -> ShooterIOSim()
            Mode.REPLAY -> object : ShooterIO {}
        }
    )

    val bindings = Bindings(this)

    fun configureFuelSim() {
        val instance = FuelSim.instance
        instance.spawnStartingFuel()
        instance.registerRobot(
            Constants.Dimensions.FULL_WIDTH.`in`(Meters),
            Constants.Dimensions.FULL_LENGTH.`in`(Meters),
            Constants.Dimensions.BUMPER_HEIGHT.`in`(Meters),
            { drive.getPose() },
            drive::getFieldRelativeSpeeds,
        )

        instance.registerIntake(
            Constants.Dimensions.FULL_LENGTH
                .div(2.0)
                .`in`(Meters),
            Constants.Dimensions.FULL_LENGTH
                .div(2.0)
                .plus(Inches.of(3.0))
                .`in`(Meters),
            -Constants.Dimensions.FULL_WIDTH
                .div(2.0)
                .minus(Inches.of(2.0))
                .`in`(Meters),
            Constants.Dimensions.FULL_WIDTH
                .div(2.0)
                .minus(Inches.of(5.0))
                .`in`(Meters),
        )

        instance.start()
        SmartDashboard.putData(
            Commands
                .runOnce(
                    {
                        FuelSim.instance.clearFuel()
                        FuelSim.instance.spawnStartingFuel()
                    },
                ).withName("Reset Fuel")
                .ignoringDisable(true),
        )
    }
}
