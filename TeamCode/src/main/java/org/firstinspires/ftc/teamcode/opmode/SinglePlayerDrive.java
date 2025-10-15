package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

@TeleOp(name="TELEOP \uD83C\uDFAE")
@Configurable
public class SinglePlayerDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        DuneStrider robot = DuneStrider.get().init(hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            robot.endLoop();
        }
    }
}
