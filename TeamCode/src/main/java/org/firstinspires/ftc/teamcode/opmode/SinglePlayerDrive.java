package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.BLUE_RELOCALIZE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.RED_RELOCALIZE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.INGEST;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.OFF;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.cmd.RelocalizeUsingLimelight3A;
import org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

// World class teleop design
@TeleOp(name = "TeleOp")
@Configurable
public class SinglePlayerDrive extends OpMode {
    private DuneStrider robot;
    private GamepadEx gamepad1Ex;
    private double teleOpMultiplier = 1.0;
    private double speedMultiplier = 1.0;
    public static double MX_SPEED_SHOT = 0.4;

    @Override
    public void init() {
        robot = DuneStrider.get().init(DuneStrider.Mode.TELEOP, MecanumDrive.lastPose, hardwareMap, telemetry);

        // load the last measured angle from auto
        //robot.turret.loadAngle(robot.turret.getLastAngle());

        robot.drive.follower.startTeleopDrive();
        gamepad1Ex = new GamepadEx(gamepad1);

        robot.eyes.setEnabled(true);
        robot.turret.setMode(Turret.Mode.DEBUG);

        teleOpMultiplier = 1.0;
        if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
            teleOpMultiplier = -1.0;
        }


        // home the turret at the beginning and at the 1 min mark
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                run(() -> robot.shooter.setIdle()),
                run(() -> robot.intake.closeLatch()),
                new HomeTurret(3.0)
        ));

        // intake bindings
        bind(GamepadKeys.Button.A,
                intakeSet(INGEST),
                intakeSet(Intake.Mode.OFF)
        );

        bind(GamepadKeys.Button.B,
            run(() -> Intake.INGEST_MOTOR_SPEED = 0.6).alongWith(intakeSet(INGEST)),
            run(() -> Intake.INGEST_MOTOR_SPEED = 1.0).alongWith(intakeSet(OFF))
        );

        bind(GamepadKeys.Button.X, intakeSet(Intake.Mode.DISCARD), intakeSet(Intake.Mode.OFF));
        // gate
        bind(GamepadKeys.Button.RIGHT_BUMPER,
                run(() -> {
                    robot.turret.setMode(Turret.Mode.PINPOINT);
                    robot.intake.openLatch();
                    robot.shooter.setMode(Shooter.Mode.DYNAMIC); // auto on
                    speedMultiplier = MX_SPEED_SHOT;
                }),
                run(() -> {
                    robot.turret.setMode(Turret.Mode.DEBUG);
                    robot.intake.closeLatch();
                    robot.shooter.setIdle(); // auto off
                    speedMultiplier = 1.0;
                })
        );


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                If(
                        run(() -> robot.drive.follower.setPose(BLUE_RELOCALIZE)),
                        run(() -> robot.drive.follower.setPose(RED_RELOCALIZE)),
                        () -> DuneStrider.alliance == DuneStrider.Alliance.BLUE
                )
        );

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
            run(() -> Turret.offset_angle += 3)
        );


        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                run(() -> Turret.offset_angle -= 3)
        );

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                run(() -> Turret.offset_angle = 0)
        );

    }

    @Override
    public void init_loop() {
            double measuredAbsAngle = 0;
            robot.turret.loadAngle(measuredAbsAngle);
    }

    @Override
    public void loop() {
        robot.endLoop();
        robot.drive.setTeleOpDrive(-gamepad1Ex.getLeftY() * teleOpMultiplier * speedMultiplier, gamepad1Ex.getLeftX() * teleOpMultiplier * speedMultiplier,  -gamepad1Ex.getRightX() * speedMultiplier);
    }

    public void bind(GamepadKeys.Button button, Command pressedCmd, Command releasedCmd) {
        gamepad1Ex.getGamepadButton(button).whenPressed(pressedCmd).whenInactive(releasedCmd);
    }
}

