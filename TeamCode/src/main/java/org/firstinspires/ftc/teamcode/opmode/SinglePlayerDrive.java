package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shootTeleOp;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto18.GATE_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.GoalAuto18.mHBA;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.BLUE_RELOCALIZE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.RED_RELOCALIZE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.INGEST;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.OFF;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.math.MathFunctions;
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


@TeleOp(name = "TeleOp")
@Config
public class SinglePlayerDrive extends OpMode {
    private DuneStrider robot;
    private GamepadEx gamepad1Ex;
    private double teleOpMultiplier = 1.0;
    private double speedMultiplier = 1.0;
    public static double MX_SPEED_SHOT = 1;

    // ── Heading lock ──────────────────────────────────────────────
    public static double headingLockTarget = Math.toRadians(mHBA(165)); // configurable via dashboard
    private PIDFController headingController;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(0.7, 0, 0.07, 0);
    private boolean headingLock = false;
    // ─────────────────────────────────────────────────────────────

    @Override
    public void init() {
        robot = DuneStrider.get().init(DuneStrider.Mode.TELEOP, MecanumDrive.lastPose, hardwareMap, telemetry);

        robot.turret.loadAngle(robot.analogEncoder.getCurrentPosition());
        robot.eyes.setEnabled(true);
        robot.drive.follower.startTeleopDrive();
        gamepad1Ex = new GamepadEx(gamepad1);
        robot.turret.setMode(Turret.Mode.PINPOINT);

        teleOpMultiplier = 1.0;
        if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
            teleOpMultiplier = -1.0;
        }

        // initialize heading PIDF controller
        headingController = new PIDFController(coefficients);

        // home the turret
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
        /*
        bind(GamepadKeys.Button.RIGHT_BUMPER,
                run(() -> {
                    robot.intake.openLatch();
                    robot.shooter.setMode(Shooter.Mode.DYNAMIC);
                    speedMultiplier = MX_SPEED_SHOT;
                }),
                new SequentialCommandGroup(
                    run(() -> {
                        robot.shooter.setMode(Shooter.Mode.RAW);
                        robot.shooter.setPower(0);
                    }),
                    waitFor(200),
                    run(() -> {
                        robot.intake.closeLatch();
                        robot.shooter.setIdle();
                        speedMultiplier = 1.0;
                    })
                )
        ); */

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                shootTeleOp()
        );

        // toggle heading lock on OPTIONS button
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                run(() -> headingLock = !headingLock)
        );

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
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
        double measuredAbsAngle = robot.analogEncoder.getCurrentPosition();
        robot.turret.loadAngle(measuredAbsAngle);
        telemetry.addData("angle", measuredAbsAngle);
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.endLoop();

        // update heading controller coefficients and error each loop
        headingController.setCoefficients(robot.drive.follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(getHeadingError());

        double turn;
        if (headingLock) {
            headingController.setCoefficients(coefficients);
            turn = headingController.run();
        } else {
            turn = -gamepad1Ex.getRightX() * speedMultiplier * 0.7;
        }

        robot.drive.setTeleOpDrive(
                -gamepad1Ex.getLeftY() * teleOpMultiplier * speedMultiplier,
                gamepad1Ex.getLeftX() * teleOpMultiplier * speedMultiplier,
                turn
        );

        telemetry.addData("Heading Lock", headingLock);
        telemetry.addData("Heading Error (deg)", Math.toDegrees(getHeadingError()));
        telemetry.addData("Target Heading (deg)", Math.toDegrees(headingLockTarget));
        telemetry.update();
    }

    private double getHeadingError() {
        return MathFunctions.getTurnDirection(robot.drive.follower.getPose().getHeading(), headingLockTarget)
                * MathFunctions.getSmallestAngleDifference(robot.drive.follower.getPose().getHeading(), headingLockTarget);
    }

    public void bind(GamepadKeys.Button button, Command pressedCmd, Command releasedCmd) {
        gamepad1Ex.getGamepadButton(button).whenPressed(pressedCmd).whenInactive(releasedCmd);
    }
}

