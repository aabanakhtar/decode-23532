package org.firstinspires.ftc.teamcode.opmode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.device.AbsoluteAnalogEncoder;

@TeleOp(name = "load em up")
public class ReadAnalogInput extends OpMode {
    AbsoluteAnalogEncoder enc;

    public void init() {
        enc = new AbsoluteAnalogEncoder(hardwareMap, "abs", 0);
    }

    public void loop() {
        telemetry.addData("pos", enc.getCurrentPosition());
        telemetry.update();
    }
}
