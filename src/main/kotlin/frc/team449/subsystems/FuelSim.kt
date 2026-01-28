package frc.team449.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class FuelSim private constructor() {
    inner class Fuel(
        var pos: Translation3d,
        var vel: Translation3d = Translation3d(),
    ) {
        fun update() {
            pos = pos.plus(vel.times(PERIOD / subticks))
            if (pos.getZ() > FUEL_RADIUS) {
                vel = vel.plus(GRAVITY.times(PERIOD / subticks))
            }
            if (abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
                vel = Translation3d(vel.getX(), vel.getY(), 0.0)
                vel = vel.times(1 - FRICTION * PERIOD / subticks)
                // pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
            }
            handleFieldCollisions()
        }

        fun handleXZLineCollision(
            lineStart: Translation3d,
            lineEnd: Translation3d,
        ) {
            if (pos.getY() < lineStart.getY() || pos.getY() > lineEnd.getY()) return // not within y range

            // Convert into 2D
            val start2d = Translation2d(lineStart.getX(), lineStart.getZ())
            val end2d = Translation2d(lineEnd.getX(), lineEnd.getZ())
            val pos2d = Translation2d(pos.getX(), pos.getZ())
            val lineVec = end2d.minus(start2d)

            // Get closest point on line
            val projected =
                start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()))

            if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm()) return // projected point not on line

            val dist = pos2d.getDistance(projected)
            if (dist > FUEL_RADIUS) return // not intersecting line

            // Back into 3D
            val normal = Translation3d(-lineVec.getY(), 0.0, lineVec.getX()).div(lineVec.getNorm())

            // Apply collision response
            pos = pos.plus(normal.times(FUEL_RADIUS - dist))
            if (vel.dot(normal) > 0) return // already moving away from line

            vel = vel.minus(normal.times((1 + FIELD_COR) * vel.dot(normal)))
        }

        fun handleFieldCollisions() {
            // floor and bumps
            for (i in FIELD_XZ_LINE_STARTS.indices) {
                handleXZLineCollision(FIELD_XZ_LINE_STARTS[i]!!, FIELD_XZ_LINE_ENDS[i]!!)
            }

            // edges
            if (pos.getX() < FUEL_RADIUS && vel.getX() < 0) {
                pos = pos.plus(Translation3d(FUEL_RADIUS - pos.getX(), 0.0, 0.0))
                vel = vel.plus(Translation3d(-(1 + FIELD_COR) * vel.getX(), 0.0, 0.0))
            } else if (pos.getX() > FIELD_LENGTH - FUEL_RADIUS && vel.getX() > 0) {
                pos = pos.plus(Translation3d(FIELD_LENGTH - FUEL_RADIUS - pos.getX(), 0.0, 0.0))
                vel = vel.plus(Translation3d(-(1 + FIELD_COR) * vel.getX(), 0.0, 0.0))
            }

            if (pos.getY() < FUEL_RADIUS && vel.getY() < 0) {
                pos = pos.plus(Translation3d(0.0, FUEL_RADIUS - pos.getY(), 0.0))
                vel = vel.plus(Translation3d(0.0, -(1 + FIELD_COR) * vel.getY(), 0.0))
            } else if (pos.getY() > FIELD_WIDTH - FUEL_RADIUS && vel.getY() > 0) {
                pos = pos.plus(Translation3d(0.0, FIELD_WIDTH - FUEL_RADIUS - pos.getY(), 0.0))
                vel = vel.plus(Translation3d(0.0, -(1 + FIELD_COR) * vel.getY(), 0.0))
            }

            // hubs
            handleHubCollisions(Hub.BLUE_HUB)
            handleHubCollisions(Hub.RED_HUB)
        }

        fun handleHubCollisions(hub: Hub) {
            hub.handleHubInteraction(this)
            val collision = hub.fuelCollideSide(this)
            if (collision.getX() != 0.0) {
                pos = pos.plus(Translation3d(collision))
                vel = vel.plus(Translation3d(-(1 + FIELD_COR) * vel.getX(), 0.0, 0.0))
            } else if (collision.getY() != 0.0) {
                pos = pos.plus(Translation3d(collision))
                vel = vel.plus(Translation3d(0.0, -(1 + FIELD_COR) * vel.getY(), 0.0))
            }

            val netCollision = hub.fuelHitNet(this)
            if (netCollision != 0.0) {
                pos = pos.plus(Translation3d(netCollision, 0.0, 0.0))
                vel = Translation3d(-vel.getX() * NET_COR, vel.getY() * NET_COR, vel.getZ())
            }
        }

        fun addImpulse(impulse: Translation3d?) {
            vel = vel.plus(impulse)
        }
    }

    private val fuels: ArrayList<Fuel> = ArrayList<Fuel>()
    private var running = false
    private var robotSupplier: Supplier<Pose2d>? = null
    private var robotSpeedsSupplier: Supplier<ChassisSpeeds>? = null
    private var robotWidth = 0.0 // size along the robot's y axis
    private var robotLength = 0.0 // size along the robot's x axis
    private var bumperHeight = 0.0
    private val intakes: ArrayList<SimIntake> = ArrayList<SimIntake>()

    /**
     * Clears the field of fuel
     */
    fun clearFuel() {
        fuels.clear()
    }

    /**
     * Spawns fuel in the neutral zone and depots
     */
    fun spawnStartingFuel() {
        // Center fuel
        val center: Translation3d = Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS)
        for (i in 0..14) {
            for (j in 0..5) {
                fuels.add(
                    Fuel(
                        center.plus(
                            Translation3d(
                                0.076 + 0.152 * j,
                                0.0254 + 0.076 + 0.152 * i,
                                0.0,
                            ),
                        ),
                    ),
                )
                fuels.add(
                    Fuel(
                        center.plus(
                            Translation3d(
                                -0.076 - 0.152 * j,
                                0.0254 + 0.076 + 0.152 * i,
                                0.0,
                            ),
                        ),
                    ),
                )
                fuels.add(
                    Fuel(
                        center.plus(
                            Translation3d(
                                0.076 + 0.152 * j,
                                -0.0254 - 0.076 - 0.152 * i,
                                0.0,
                            ),
                        ),
                    ),
                )
                fuels.add(
                    Fuel(
                        center.plus(
                            Translation3d(
                                -0.076 - 0.152 * j,
                                -0.0254 - 0.076 - 0.152 * i,
                                0.0,
                            ),
                        ),
                    ),
                )
            }
        }

        // Depots
        for (i in 0..2) {
            for (j in 0..3) {
                fuels.add(Fuel(Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)))
                fuels.add(Fuel(Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)))
                fuels.add(
                    Fuel(
                        Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS),
                    ),
                )
                fuels.add(
                    Fuel(
                        Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS),
                    ),
                )
            }
        }
    }

    /**
     * Adds array of `Translation3d`'s to NetworkTables at "AdvantageKit/RealOutputs/Fuel Simulation/Fuels"
     */

    fun logFuels() {
        Logger.recordOutput(
            "Fuel Simulation/Fuels",
            *fuels.map { it.pos }.toTypedArray(),
        )
    }

    /**
     * Start the simulation. `updateSim` must still be called every loop
     */
    fun start() {
        running = true
    }

    /**
     * Pause the simulation.
     */
    fun stop() {
        running = false
    }

    /**
     * Sets the number of physics iterations per loop (0.02s)
     * @param subticks
     */
    fun setSubticks(subticks: Int) {
        Companion.subticks = subticks
    }

    /**
     * Registers a robot with the fuel simulator
     * @param width from left to right (y-axis)
     * @param length from front to back (x-axis)
     * @param bumperHeight
     * @param poseSupplier
     * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
     */
    fun registerRobot(
        width: Double,
        length: Double,
        bumperHeight: Double,
        poseSupplier: Supplier<Pose2d>?,
        fieldSpeedsSupplier: Supplier<ChassisSpeeds>?,
    ) {
        this.robotSupplier = poseSupplier
        this.robotSpeedsSupplier = fieldSpeedsSupplier
        this.robotWidth = width
        this.robotLength = length
        this.bumperHeight = bumperHeight
    }

    /**
     * To be called periodically
     * Will do nothing if sim is not running
     */
    fun updateSim() {
        if (!running) return

        stepSim()
    }

    /**
     * Run the simulation forward 1 time step (0.02s)
     */
    fun stepSim() {
        for (i in 0..<subticks) {
            for (fuel in fuels) {
                fuel.update()
            }

            handleFuelCollisions(fuels)

            if (robotSupplier != null) {
                handleRobotCollisions(fuels)
                handleIntakes(fuels)
            }
        }

        logFuels()
    }

    /**
     * Adds a fuel onto the field
     * @param pos Position to spawn at
     * @param vel Initial velocity vector
     */
    fun spawnFuel(
        pos: Translation3d,
        vel: Translation3d,
    ) {
        fuels.add(Fuel(pos, vel))
    }

    private fun handleRobotCollision(
        fuel: Fuel,
        robot: Pose2d,
        robotVel: Translation2d,
    ) {
        val relativePos =
            Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                .relativeTo(robot)
                .getTranslation()

        if (fuel.pos.getZ() > bumperHeight) return // above bumpers

        val distanceToBottom: Double = -FUEL_RADIUS - robotLength / 2 - relativePos.getX()
        val distanceToTop: Double = -FUEL_RADIUS - robotLength / 2 + relativePos.getX()
        val distanceToRight: Double = -FUEL_RADIUS - robotWidth / 2 - relativePos.getY()
        val distanceToLeft: Double = -FUEL_RADIUS - robotWidth / 2 + relativePos.getY()

        // not inside robot
        if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0) return

        var posOffset: Translation2d
        // find minimum distance to side and send corresponding collision response
        if ((distanceToBottom >= distanceToTop && distanceToBottom >= distanceToRight && distanceToBottom >= distanceToLeft)) {
            posOffset = Translation2d(distanceToBottom, 0.0)
        } else if ((distanceToTop >= distanceToBottom && distanceToTop >= distanceToRight && distanceToTop >= distanceToLeft)) {
            posOffset = Translation2d(-distanceToTop, 0.0)
        } else if ((distanceToRight >= distanceToBottom && distanceToRight >= distanceToTop && distanceToRight >= distanceToLeft)) {
            posOffset = Translation2d(0.0, distanceToRight)
        } else {
            posOffset = Translation2d(0.0, -distanceToLeft)
        }

        posOffset = posOffset.rotateBy(robot.getRotation())
        fuel.pos = fuel.pos.plus(Translation3d(posOffset))
        val normal = posOffset.div(posOffset.getNorm())
        if (fuel.vel.toTranslation2d().dot(normal) < 0) {
            fuel.addImpulse(
                Translation3d(normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))),
            )
        }
        if (robotVel.dot(normal) > 0) fuel.addImpulse(Translation3d(normal.times(robotVel.dot(normal))))
    }

    private fun handleRobotCollisions(fuels: ArrayList<Fuel>) {
        val robot = robotSupplier!!.get()
        val speeds = robotSpeedsSupplier!!.get()
        val robotVel = Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)

        for (fuel in fuels) {
            handleRobotCollision(fuel, robot, robotVel)
        }
    }

    private fun handleIntakes(fuels: ArrayList<Fuel>) {
        val robot = robotSupplier!!.get()
        for (intake in intakes) {
            var i = 0
            while (i < fuels.size) {
                if (intake.shouldIntake(fuels.get(i), robot)) {
                    fuels.removeAt(i)
                    i--
                }
                i++
            }
        }
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     * @param intakeCallback Function to call when a fuel is intaked
     */
    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     */

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     */
    @JvmOverloads
    fun registerIntake(
        xMin: Double,
        xMax: Double,
        yMin: Double,
        yMax: Double,
        ableToIntake: BooleanSupplier = BooleanSupplier { true },
        intakeCallback: Runnable = Runnable {},
    ) {
        intakes.add(SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback))
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param intakeCallback Function to call when a fuel is intaked
     */
    fun registerIntake(
        xMin: Double,
        xMax: Double,
        yMin: Double,
        yMax: Double,
        intakeCallback: Runnable,
    ) {
        registerIntake(xMin, xMax, yMin, yMax, BooleanSupplier { true }, intakeCallback)
    }

    class Hub private constructor(
        private val center: Translation2d,
        private val exit: Translation3d,
        private val exitVelXMult: Int,
    ) {
        /**
         * Get the current count of fuel scored in this hub
         * @return
         */
        var score: Int = 0
            private set

        fun handleHubInteraction(fuel: Fuel) {
            if (didFuelScore(fuel)) {
                fuel.pos = exit
                fuel.vel = this.dispersalVelocity
                score++
            }
        }

        private fun didFuelScore(fuel: Fuel): Boolean =
            fuel.pos
                .toTranslation2d()
                .getDistance(center) <= ENTRY_RADIUS && fuel.pos.getZ() <= ENTRY_HEIGHT && fuel.pos
                .minus(
                    fuel.vel.times(
                        PERIOD / subticks,
                    ),
                ).getZ() > ENTRY_HEIGHT

        private val dispersalVelocity: Translation3d
            get() =
                Translation3d(
                    exitVelXMult * (Math.random() + 0.1) * 1.5,
                    Math.random() * 2 - 1,
                    0.0,
                )

        /**
         * Reset this hub's score to 0
         */
        fun resetScore() {
            score = 0
        }

        fun fuelCollideSide(fuel: Fuel): Translation2d {
            if (fuel.pos.getZ() > ENTRY_HEIGHT - 0.1) return Translation2d() // above hub

            val distanceToLeft: Double = center.getX() - SIDE / 2 - FUEL_RADIUS - fuel.pos.getX()
            val distanceToRight: Double = fuel.pos.getX() - center.getX() - SIDE / 2 - FUEL_RADIUS
            val distanceToTop: Double = center.getY() - SIDE / 2 - FUEL_RADIUS - fuel.pos.getY()
            val distanceToBottom: Double = fuel.pos.getY() - center.getY() - SIDE / 2 - FUEL_RADIUS

            // not inside hub
            if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0) return Translation2d()

            // find minimum distance to side and send corresponding collision response
            if (fuel.pos.getX() < center.getX() - SIDE / 2 ||
                (distanceToLeft >= distanceToRight && distanceToLeft >= distanceToTop && distanceToLeft >= distanceToBottom)
            ) {
                return Translation2d(distanceToLeft, 0.0)
            } else if (fuel.pos.getX() >= center.getX() + SIDE / 2 ||
                (distanceToRight >= distanceToLeft && distanceToRight >= distanceToTop && distanceToRight >= distanceToBottom)
            ) {
                return Translation2d(-distanceToRight, 0.0)
            } else if (fuel.pos.getY() > center.getY() + SIDE / 2 ||
                (distanceToTop >= distanceToLeft && distanceToTop >= distanceToRight && distanceToTop >= distanceToBottom)
            ) {
                return Translation2d(0.0, -distanceToTop)
            } else {
                return Translation2d(0.0, distanceToBottom)
            }
        }

        fun fuelHitNet(fuel: Fuel): Double {
            if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN) return 0.0
            if (fuel.pos.getY() > center.getY() + NET_WIDTH / 2 || fuel.pos.getY() < center.getY() - NET_WIDTH / 2) return 0.0
            if (fuel.pos.getX() > center.getX() + NET_OFFSET * exitVelXMult) {
                return max(0.0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() - FUEL_RADIUS))
            } else {
                return min(0.0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() + FUEL_RADIUS))
            }
        }

        companion object {
            val BLUE_HUB: Hub = Hub(Translation2d(4.61, FIELD_WIDTH / 2), Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1)
            val RED_HUB: Hub =
                Hub(
                    Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
                    Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
                    -1,
                )

            private const val ENTRY_HEIGHT = 1.83
            private const val ENTRY_RADIUS = 0.56

            private const val SIDE = 1.2

            private const val NET_HEIGHT_MAX = 3.057
            private const val NET_HEIGHT_MIN = 1.5
            private val NET_OFFSET = SIDE / 2 + 0.261
            private const val NET_WIDTH = 1.484
        }
    }

    private inner class SimIntake(
        var xMin: Double,
        var xMax: Double,
        var yMin: Double,
        var yMax: Double,
        var ableToIntake: BooleanSupplier,
        var callback: Runnable,
    ) {
        fun shouldIntake(
            fuel: Fuel,
            robotPose: Pose2d?,
        ): Boolean {
            if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight) return false

            val fuelRelativePos =
                Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                    .relativeTo(robotPose)
                    .getTranslation()

            val result =
                fuelRelativePos.getX() >= xMin && fuelRelativePos.getX() <= xMax && fuelRelativePos.getY() >= yMin &&
                    fuelRelativePos.getY() <= yMax
            if (result) {
                callback.run()
            }
            return result
        }
    }

    companion object {
        private const val PERIOD = 0.02 // sec
        private var subticks = 5
        private val GRAVITY = Translation3d(0.0, 0.0, -9.81) // m/s^2
        private val FIELD_COR = sqrt(22 / 51.5) // coefficient of restitution with the field
        private const val FUEL_COR = 0.05 // 0.5 // coefficient of restitution with another fuel
        private const val NET_COR = 0.2 // coefficient of restitution with the net
        private const val ROBOT_COR = 0.1 // coefficient of restitution with a robot
        private const val FUEL_RADIUS = 0.075
        private const val FIELD_LENGTH = 16.51
        private const val FIELD_WIDTH = 8.04
        private const val FRICTION = 0.31 // proportion of horizontal velocity to lose per second while on ground

        var instance: FuelSim = FuelSim()
            /**
             * Returns a singleton instance of frc.team449.FuelSim
             */
            get() {

                return field
            }
            private set

        private val FIELD_XZ_LINE_STARTS =
            arrayOf<Translation3d?>(
                Translation3d(0.0, 0.0, 0.0),
                Translation3d(3.96, 1.57, 0.0),
                Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0.0),
                Translation3d(4.61, 1.57, 0.165),
                Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
                Translation3d(FIELD_LENGTH - 5.18, 1.57, 0.0),
                Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0.0),
                Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
                Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
            )

        private val FIELD_XZ_LINE_ENDS =
            arrayOf<Translation3d?>(
                Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0.0),
                Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
                Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
                Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0.0),
                Translation3d(5.18, FIELD_WIDTH - 1.57, 0.0),
                Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
                Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
                Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0.0),
                Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0.0),
            )

        private fun handleFuelCollision(
            a: Fuel,
            b: Fuel,
        ) {
            var normal = a.pos.minus(b.pos)
            var distance = normal.getNorm()
            if (distance == 0.0) {
                normal = Translation3d(1.0, 0.0, 0.0)
                distance = 1.0
            }
            normal = normal.div(distance)
            val impulse: Double = 0.5 * (1 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal))
            val intersection: Double = FUEL_RADIUS * 2 - distance
            a.pos = a.pos.plus(normal.times(intersection / 2))
            b.pos = b.pos.minus(normal.times(intersection / 2))
            a.addImpulse(normal.times(impulse))
            b.addImpulse(normal.times(-impulse))
        }

        private fun handleFuelCollisions(fuels: ArrayList<Fuel>) {
            for (i in 0..<fuels.size - 1) {
                for (j in i + 1..<fuels.size) {
                    if (fuels.get(i).pos.getDistance(fuels.get(j).pos) < FUEL_RADIUS * 2) {
                        handleFuelCollision(fuels.get(i), fuels.get(j))
                    }
                }
            }
        }
    }
}
