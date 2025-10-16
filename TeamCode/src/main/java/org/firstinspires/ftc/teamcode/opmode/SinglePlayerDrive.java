package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@TeleOp(name = "TELEOP 🎮")
@Configurable
public class SinglePlayerDrive extends OpMode {
    private DuneStrider robot;

    @Override
    public void init() {
        robot = DuneStrider.get().init(new Pose(), hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        robot.endLoop();
    }

    @Override
    public void loop() {
        robot.drive.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        robot.endLoop();
    }

}

