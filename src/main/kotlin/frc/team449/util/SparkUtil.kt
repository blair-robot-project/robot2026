package frc.team449.util

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import java.util.function.Consumer
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import java.util.function.Function
import java.util.function.Supplier

object SparkUtil {
/** Stores whether any error was has been detected by other utility methods. */
    var sparkStickyFault: Boolean = false

/** Processes a value from a Spark only if the value is valid. */
    fun ifOk(spark: SparkBase, supplier: DoubleSupplier, consumer: DoubleConsumer) {
        val value = supplier.asDouble
        if (spark.lastError == REVLibError.kOk) {
            consumer.accept(value)
        } else {
            sparkStickyFault = true
        }
    }

/** Processes a value from a Spark only if the value is valid. */
    fun ifOk(
        spark: SparkBase,
        suppliers: Array<DoubleSupplier>,
        consumer: Consumer<DoubleArray>
    ) {
        val values = DoubleArray(suppliers.size)
        for (i in suppliers.indices) {
            values[i] = suppliers[i].asDouble
            if (spark.lastError != REVLibError.kOk) {
                sparkStickyFault = true
                return
            }
        }
        consumer.accept(values)
    }

/** Return a value from a Spark (or the default if the value is invalid). */
    fun ifOkOrDefault(
        spark: SparkBase,
        supplier: DoubleSupplier,
        defaultValue: Double
    ): Double {
        val value = supplier.asDouble
        if (spark.lastError == REVLibError.kOk) {
            return value
        } else {
            sparkStickyFault = true
            return defaultValue
        }
    }

/** Return a processed set of values from a Spark (or the default if one of the values is invalid). */
    fun ifOkOrDefault(
        spark: SparkBase,
        suppliers: Array<DoubleSupplier>,
        transformer: Function<Array<Double>, Double>,
        defaultValue: Double
    ): Double {
        val values = arrayOf<Double>()
        for (i in suppliers.indices) {
            values[i] = suppliers[i].asDouble
            if (spark.lastError != REVLibError.kOk) {
                sparkStickyFault = true
                return defaultValue
            }
        }
        return transformer.apply(values)
    }

/** Attempts to run the command until no error is produced. */
    fun tryUntilOk(spark: SparkBase, maxAttempts: Int, command: Supplier<REVLibError>) {
        for (i in 0 until maxAttempts) {
            val error = command.get()
            if (error == REVLibError.kOk) {
                break
            } else {
                sparkStickyFault = true
            }
        }
    }
}
