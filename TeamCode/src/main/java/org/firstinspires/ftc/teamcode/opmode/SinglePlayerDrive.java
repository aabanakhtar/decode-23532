package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.INGEST;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

// World class teleop design
@TeleOp(name = "Field Centric TeleOp 🎮", group = "manual")
@Configurable
public class SinglePlayerDrive extends OpMode {
    private DuneStrider robot;
    private GamepadEx gamepad1Ex;

    @Override
    public void init() {
        robot = DuneStrider.get().init(new Pose(72, 72, 0), hardwareMap, telemetry);
        robot.drive.follower.startTeleopDrive();
        gamepad1Ex = new GamepadEx(gamepad1);

        // initialization
        CommandScheduler.getInstance().schedule(new HomeTurret(3));

        // intake bindings
        bind(GamepadKeys.Button.A,
                new SequentialCommandGroup(
                    // close the latch and run the intake
                    intakeSet(INGEST)
                ),
                intakeSet(Intake.Mode.OFF)
        );

        bind(GamepadKeys.Button.X, intakeSet(Intake.Mode.DISCARD), intakeSet(Intake.Mode.OFF));

        bind(GamepadKeys.Button.RIGHT_BUMPER,
                    new SequentialCommandGroup(
                            run(() -> robot.shooter.setVelocity(-1100)),
                            waitFor((long)Intake.INTAKE_LATCH_DELAY),
                            run(() -> robot.intake.openLatch())
                    ),
            run(() -> robot.intake.closeLatch())
                    .alongWith(run(() -> robot.shooter.setVelocity(-600)))
        );

        // reset localizer bindings
        bind(GamepadKeys.Button.START, run(() -> robot.drive.resetHeading(0)), nothing());
        bind(GamepadKeys.Button.SHARE, new HomeTurret(2), nothing());

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> robot.drive.follower.setPose(new Pose(72, 72, 0)))
        );
    }

    @Override
    public void loop() {
        robot.endLoop();
        robot.drive.setTeleOpDrive(gamepad1Ex.getLeftY(), -gamepad1Ex.getLeftX(), -gamepad1Ex.getRightX());
    }

    public void bind(GamepadKeys.Button button, Command pressedCmd, Command releasedCmd) {
        gamepad1Ex.getGamepadButton(button).whenPressed(pressedCmd).whenInactive(releasedCmd);
    }
}

