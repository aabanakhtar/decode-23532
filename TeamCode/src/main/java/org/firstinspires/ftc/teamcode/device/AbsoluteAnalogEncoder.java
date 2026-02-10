package org.firstinspires.ftc.teamcode.device;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class AbsoluteAnalogEncoder {
    private final AnalogInput encoder;
    public double offset;

    public AbsoluteAnalogEncoder(HardwareMap map, String id, double off) {
        encoder = map.get(AnalogInput.class, id);
        offset = off;
    }

    public double getCurrentPosition() {
        return (getVoltage() - 0.017) / (3.244 - 0.017) * 360 - offset;
    }

    public double getCurrentPositionNoOffset() {
        return (getVoltage() - 0.017) / (3.244 - 0.017) * 360;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }
}