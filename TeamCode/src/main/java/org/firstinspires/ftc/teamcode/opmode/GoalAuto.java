package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.fork;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shoot;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE_HP_ZONE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT4;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.OPEN_GATE_OPEN;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.SCORE_CONTROL_POINT3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.START_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.UNIVERSAL_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.heading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.cmd.HomeTurret;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Global Goal Autonomous", group = "auto", preselectTeleOp = "TeleOp")
public class GoalAuto extends OpMode {
    public static double TRANSFER_DELAY = 800.0;

    private DuneStrider robot;
    private PathChain shootPreload;
    private PathChain intakeRow1, intakeRow2, intakeRow3;
    private PathChain scoreRow1, scoreRow2, scoreRow3;
    private PathChain intakeHPZone, scoreHPZone;
    public static int nRows = 4;

    @Override
    public void init() {
        Pose startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? START_PRELOAD.setHeading(0) : START_PRELOAD.mirror().setHeading(heading(180));

        robot = DuneStrider.get().init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);
        Follower follower = robot.drive.follower;

        if (DuneStrider.alliance == DuneStrider.Alliance.BLUE) buildPathChains(follower);
        else buildPathChainsRed(follower);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new HomeTurret(0.1),
                        execPreload(),
                        If(execRow1(), nothing(), () -> nRows >= 1),
                        If(execRow2(), nothing(), () -> nRows >= 2),
                        If(execRow3(), nothing(), () -> nRows >= 3),
                        If(execHPZone(), nothing(), () -> nRows >= 4),
                        run(() -> robot.shooter.setVelocity(0))

                )
        );
    }

    @Override
    public void init_loop() {
        telemetry.addLine("====DUNESTRIDER PRE-MATCH Config=====");
        telemetry.addData("|| > ALLIANCE:", DuneStrider.alliance.toString());
        telemetry.addData("|| > ROWS:", nRows);
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.endLoop();
    }

    private Command execPreload() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.openLatch()),
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                go(robot.drive.follower, shootPreload, 1.0),
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRow1() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                go(robot.drive.follower, intakeRow1, 1.0),

                intakeSet(Intake.Mode.OFF),
                // go home and score
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                go(robot.drive.follower, scoreRow1, 1.0),
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRow2() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                // turn on the intake and eat up the balls
                intakeSet(Intake.Mode.INGEST),
                go(robot.drive.follower, intakeRow2, 1),
                intakeSet(Intake.Mode.OFF),

                // go home and score
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                go(robot.drive.follower, scoreRow2, 1),
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execRow3() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),

                // turn on the intake and eat up the balls
                intakeSet(Intake.Mode.INGEST),
                go(robot.drive.follower, intakeRow3, 1),

                intakeSet(Intake.Mode.OFF),

                // go home and score
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                go(robot.drive.follower, scoreRow3, 1),
                shoot((long) TRANSFER_DELAY)
        );
    }

    private Command execHPZone() {
        return new SequentialCommandGroup(
                fork(
                        run(() -> robot.intake.closeLatch()),
                        // turn on the intake and eat up the balls
                        intakeSet(Intake.Mode.INGEST)
                ),

                go(robot.drive.follower, intakeHPZone, 1),

                intakeSet(Intake.Mode.OFF),

                // go home and score
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                go(robot.drive.follower, scoreHPZone, 1),
                shoot((long) TRANSFER_DELAY)
        );
    }

    private void buildPathChains(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierLine(START_PRELOAD, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(0, heading(-90))
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                        UNIVERSAL_SCORE_TARGET,
                        INTAKE_CONTROL_POINT,
                        END_INTAKE_START_SCORE))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(OPEN_GATE_OPEN, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(heading(180), heading(-90))
                .build();

        intakeRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                UNIVERSAL_SCORE_TARGET,
                                INTAKE_CONTROL_POINT2,
                                END_INTAKE_START_SCORE2
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                OPEN_GATE_OPEN,
                                new Pose(58, 73),
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(UNIVERSAL_SCORE_TARGET,
                                INTAKE_CONTROL_POINT3,
                                END_INTAKE_START_SCORE3)
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE3,
                                SCORE_CONTROL_POINT3,
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        intakeHPZone = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                UNIVERSAL_SCORE_TARGET,
                                INTAKE_CONTROL_POINT4,
                                END_INTAKE_START_SCORE_HP_ZONE
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreHPZone = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE_HP_ZONE,
                                INTAKE_CONTROL_POINT4,
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    private void buildPathChainsRed(Follower follower) {
        // TODO: fill
    }

    @Override
    public void stop() {
        Turret.autonomousEncoderOffset = robot.shooterTurret.getCurrentPosition();
    }

    private double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}