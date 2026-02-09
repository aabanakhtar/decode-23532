package org.firstinspires.ftc.teamcode.opmode.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.device.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@TeleOp(name = "load em up")
public class ReadAnalogInput extends OpMode {
    AbsoluteAnalogEncoder enc;
    MultipleTelemetry t = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    public void init() {
        enc = new AbsoluteAnalogEncoder(hardwareMap, "abs", 0);
    }

    public void loop() {
        t.addData("pos", enc.getCurrentPositionNoOffset());
        t.addData("voltage", enc.getVoltage());
        DuneStrider.TURRET_ENCODER_OFFSET = -enc.getCurrentPositionNoOffset();
        t.update();
    }
}
