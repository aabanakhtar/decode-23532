package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.seq;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.cmd.SetShooter;
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
        robot = DuneStrider.get().init(MecanumDrive.lastPose, hardwareMap, telemetry);
        robot.drive.follower.startTeleopDrive();
        gamepad1Ex = new GamepadEx(gamepad1);

        // initialization
        CommandScheduler.getInstance().schedule(new HomeTurret());

        // intake bindings
        bind(GamepadKeys.Button.A,
                seq(
                        // close the latch and run the intake
                        run(() -> robot.intake.closeLatch()).alongWith(waitFor((long)Intake.INTAKE_LATCH_DELAY)),
                        intakeSet(Intake.Mode.INGEST)),
                intakeSet(Intake.Mode.OFF));

        bind(GamepadKeys.Button.X, intakeSet(Intake.Mode.DISCARD), intakeSet(Intake.Mode.OFF));

        // shooter bindings
        bind(GamepadKeys.Button.RIGHT_BUMPER,
            seq(
                    run(() -> robot.intake.openLatch()).alongWith(waitFor((long)Intake.INTAKE_LATCH_DELAY)),
                    new SetShooter(Shooter.Mode.RAW, -1.0, false)
            ),
            new SetShooter(Shooter.Mode.RAW, 0.7, false)
        );

        // reset localizer bindings
        bind(GamepadKeys.Button.START, run(() -> robot.drive.resetHeading(0)), nothing());
        bind(GamepadKeys.Button.SHARE, run(() -> robot.drive.follower.setPose(new Pose(7,7,0))), nothing());

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading(0))
        );
    }

    @Override
    public void loop() {
        robot.endLoop();
        robot.drive.setTeleOpDrive(gamepad1Ex.getLeftY(), -gamepad1Ex.getLeftX(), -gamepad1Ex.getRightX());
    }

    public void bind(GamepadKeys.Button button, Command pressedCmd, Command releasedCmd) {
        gamepad1Ex.getGamepadButton(button).whenPressed(pressedCmd).whenReleased(releasedCmd);
    }
}

