package frc.team449.util

import frc.team449.Constants
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.Arrays
import java.util.function.Consumer
import java.util.function.DoubleSupplier

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
@Suppress("unused")
class LoggedTunableNumber(dashboardKey: String) : DoubleSupplier {
    private val key: String = "$TABLE_KEY/$dashboardKey"
    private var hasDefault = false
    private var defaultValue = 0.0
    private lateinit var dashboardNumber: LoggedNetworkNumber
    private val lastHasChangedValues: MutableMap<Int, Double> = HashMap()

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    constructor(dashboardKey: String, defaultValue: Double) : this(dashboardKey) {
        initDefault(defaultValue)
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    fun initDefault(defaultValue: Double) {
        if (!hasDefault) {
            hasDefault = true
            this.defaultValue = defaultValue
            if (Constants.TUNING_MODE) {
                dashboardNumber = LoggedNetworkNumber(key, defaultValue)
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    fun get(): Double {
        return if (!hasDefault) {
            0.0
        } else {
            if (Constants.TUNING_MODE) dashboardNumber.get() else defaultValue
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     * objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     * otherwise.
     */
    fun hasChanged(id: Int): Boolean {
        val currentValue = get()
        val lastValue = lastHasChangedValues[id]
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues[id] = currentValue
            return true
        }

        return false
    }

    override fun getAsDouble(): Double {
        return get()
    }

    companion object {
        private const val TABLE_KEY = "/Tuning"

        /**
         * Runs action if any of the tunableNumbers have changed
         *
         * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
         * objects. Recommended approach is to pass the result of "hashCode()"
         * @param action Callback to run when any of the tunable numbers have changed. Access tunable
         * numbers in order inputted in method
         * @param tunableNumbers All tunable numbers to check
         */
        fun ifChanged(
            id: Int,
            action: Consumer<DoubleArray>,
            vararg tunableNumbers: LoggedTunableNumber
        ) {
            if (Arrays.stream(tunableNumbers).anyMatch { tunableNumber: LoggedTunableNumber -> tunableNumber.hasChanged(id) }) {
                action.accept(Arrays.stream(tunableNumbers).mapToDouble { obj: LoggedTunableNumber -> obj.get() }.toArray())
            }
        }

        /** Runs action if any of the tunableNumbers have changed  */
        fun ifChanged(id: Int, action: Runnable, vararg tunableNumbers: LoggedTunableNumber) {
            ifChanged(id, { values: DoubleArray -> action.run() }, *tunableNumbers)
        }
    }
}
