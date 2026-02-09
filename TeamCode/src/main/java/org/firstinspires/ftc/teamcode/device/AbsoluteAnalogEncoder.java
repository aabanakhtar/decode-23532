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
        double pos = -((getVoltage() / 3.244) * 360) - offset;
        return -AngleUnit.normalizeDegrees(pos);
    }

    public double getCurrentPositionNoOffset() {
        double pos = -(getVoltage() / 3.244) * 360;
        return -AngleUnit.normalizeDegrees(pos);
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }
}