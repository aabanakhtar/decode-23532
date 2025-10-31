package org.firstinspires.ftc.teamcode.subsystem;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class GlobalSensorMap extends SubsystemBase {
    private static final HashMap<String, Double> sensorReadings = new HashMap<>();
    private static final HashMap<String, DoubleSupplier> sensorProvider = new HashMap<>();

    public static void createSensor(String key, DoubleSupplier supplier) {
        sensorProvider.put(key, supplier);
        sensorReadings.put(key, supplier.getAsDouble());
    }

    public static void set(String key, double value) {
        sensorReadings.put(key, value);
    }

    @Override
    public void periodic() {

    }
}
