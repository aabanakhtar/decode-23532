package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.INGEST;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

// World class teleop design
@TeleOp(name = "TeleOp", group = "manual")
@Configurable
public class SinglePlayerDrive extends OpMode {
    private DuneStrider robot;
    private GamepadEx gamepad1Ex;
    private double teleOpMultiplier = 1.0;

    @Override
    public void init() {
        robot = DuneStrider.get().init(MecanumDrive.lastPose, hardwareMap, telemetry);
        robot.drive.follower.startTeleopDrive();
        gamepad1Ex = new GamepadEx(gamepad1);

        teleOpMultiplier = 1.0;
        if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
            teleOpMultiplier = -1.0;
        }

        // initialization

        // home the turret at the beginning and at the 1 min mark
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new HomeTurret(1.0)/*,
                new WaitCommand(60000),
                new HomeTurret(1.5)*/
        ));

        // intake bindings
        bind(GamepadKeys.Button.A,
                intakeSet(INGEST),
                intakeSet(Intake.Mode.OFF)
        );

        bind(GamepadKeys.Button.B,
            run(() -> Intake.INGEST_MOTOR_SPEED = 0.7),
            run(() -> Intake.INGEST_MOTOR_SPEED = 1.0)
        );

        bind(GamepadKeys.Button.X, intakeSet(Intake.Mode.DISCARD), intakeSet(Intake.Mode.OFF));
        // gate
        bind(GamepadKeys.Button.RIGHT_BUMPER,
                run(() -> {
                    robot.intake.openLatch();
                    robot.shooter.setMode(Shooter.Mode.DYNAMIC); // auto on
                }),
                run(() -> {
                    robot.intake.closeLatch();
                    robot.shooter.setIdle(); // auto off
                })
        );

        // reset localizer bindings
        bind(GamepadKeys.Button.START, run(() -> robot.drive.resetHeading(0)), nothing());
        bind(GamepadKeys.Button.SHARE, new HomeTurret(2), nothing());

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                If(
                        run(() -> robot.drive.follower.setPose(new Pose(72, 72, 0))),
                        run(() -> robot.drive.follower.setPose(new Pose(72, 72, Math.toRadians(180)))),
                        () -> DuneStrider.alliance == DuneStrider.Alliance.RED
                )
        );
    }

    @Override
    public void loop() {
        robot.endLoop();
        robot.drive.setTeleOpDrive(-gamepad1Ex.getLeftY() * teleOpMultiplier, gamepad1Ex.getLeftX() * teleOpMultiplier,  -gamepad1Ex.getRightX());
    }

    public void bind(GamepadKeys.Button button, Command pressedCmd, Command releasedCmd) {
        gamepad1Ex.getGamepadButton(button).whenPressed(pressedCmd).whenInactive(releasedCmd);
    }
}

