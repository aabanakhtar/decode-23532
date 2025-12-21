package org.firstinspires.ftc.teamcode.device;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwyftRanger {
    private AnalogInput analogInput;

    public SwyftRanger(HardwareMap map, String name) {
        analogInput = map.get(AnalogInput.class, name);
    }

    public double getDistance() {
        return (analogInput.getVoltage()) - 0;
    }
}
