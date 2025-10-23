package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cmd.SetShooter;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp(name = "TELEOP 🎮", group = "manual")
@Configurable
public class SinglePlayerDrive extends OpMode {
    private DuneStrider robot;
    private GamepadEx gamepad1Ex;

    @Override
    public void init() {
        robot = DuneStrider.get().init(new Pose(0, 0), hardwareMap, telemetry);
        gamepad1Ex = new GamepadEx(gamepad1);

        /* INTAKE BINDING */
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.setMode(Intake.Mode.INGEST)),
                        new SetShooter(Shooter.Mode.RAW, -0.3)
                )
        ).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intake.setMode(Intake.Mode.OFF)),
                        new SetShooter(Shooter.Mode.RAW, 0.0)
                )
        );

        // shooter
        new Trigger(() -> gamepad1Ex.gamepad.right_trigger > 0.5f).whenActive(
                new ParallelCommandGroup()
        ).whenInactive(
                new ParallelCommandGroup()
        );

        /* IMU RESET for Heading Control */
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading(0))
        );

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

