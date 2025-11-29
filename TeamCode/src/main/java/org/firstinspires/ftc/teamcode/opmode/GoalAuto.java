package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.fork;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shoot;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.CONTROL1_SCORE_ROW2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE3;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_LINEUP_START_INTAKE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_LINEUP_START_INTAKE2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_LINEUP_START_INTAKE3;
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

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Global Goal Autonomous", group = "auto", preselectTeleOp = "TeleOp")
public class GoalAuto extends OpMode {
    public static double TRANSFER_DELAY = 500.0;

    private DuneStrider robot;
    private PathChain shootPreload;
    private PathChain lineUpRow1, lineUpRow2, lineUpRow3;
    private PathChain intakeRow1, intakeRow2, intakeRow3;
    private PathChain scoreRow1, scoreRow2, scoreRow3;
    public static int nRows = 3;

    @Override
    public void init() {
        Pose startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? START_PRELOAD.setHeading(0) : START_PRELOAD.mirror().setHeading(heading(180));

        robot = DuneStrider.get().init(startPose, hardwareMap, telemetry);
        Follower follower = robot.drive.follower;

        if (DuneStrider.alliance == DuneStrider.Alliance.BLUE) buildPathChains(follower);
        else buildPathChainsRed(follower);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new HomeTurret(1),
                        execPreload(),
                        If(execRow1(), nothing(), () -> nRows >= 1),
                        If(execRow2(), nothing(), () -> nRows >= 2),
                        If(execRow3(), nothing(), () -> nRows >= 3),
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
                fork(
                        run(() -> robot.intake.closeLatch()),
                        go(robot.drive.follower, lineUpRow1, 1.0)
                ),

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
                fork(
                        go(robot.drive.follower, lineUpRow2, 1.0),
                        run(() -> robot.intake.closeLatch())
                ),

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
                fork(
                        go(robot.drive.follower, lineUpRow3, 1.0),
                        run(() -> robot.intake.closeLatch())
                ),

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

    private void buildPathChains(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierLine(START_PRELOAD, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(0, heading(-90))
                .build();

        lineUpRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE))
                .setLinearHeadingInterpolation(heading(-90), heading(180))
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE, END_INTAKE_START_SCORE))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_INTAKE_START_SCORE, UNIVERSAL_SCORE_TARGET))
                .setLinearHeadingInterpolation(heading(180), heading(-90))
                .build();


        lineUpRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE2)
                )
                .setLinearHeadingInterpolation(heading(-90), heading(180))
                .build();

        intakeRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(END_LINEUP_START_INTAKE2, END_INTAKE_START_SCORE2)
                )
                .setLinearHeadingInterpolation(heading(180), heading(180))
                .build();

        scoreRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE2,
                                CONTROL1_SCORE_ROW2,
                                UNIVERSAL_SCORE_TARGET
                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(-90))
                .build();

        lineUpRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(UNIVERSAL_SCORE_TARGET, END_LINEUP_START_INTAKE3)
                )
                .setLinearHeadingInterpolation(heading(-90), heading(180))
                .build();

        intakeRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(END_LINEUP_START_INTAKE3, END_INTAKE_START_SCORE3)
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                END_INTAKE_START_SCORE3,
                                UNIVERSAL_SCORE_TARGET

                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(-90))
                .build();
    }

    private void buildPathChainsRed(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(new BezierLine(START_PRELOAD.mirror(), UNIVERSAL_SCORE_TARGET.mirror()))
                .setLinearHeadingInterpolation(mirrorHeading(0), mirrorHeading(heading(-90)))
                .build();

        lineUpRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET.mirror(), END_LINEUP_START_INTAKE.mirror()))
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(-90)),
                        mirrorHeading(heading(180))
                )
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE.mirror(), END_INTAKE_START_SCORE.mirror()))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_INTAKE_START_SCORE.mirror(), UNIVERSAL_SCORE_TARGET.mirror()))
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(180)),
                        mirrorHeading(heading(-90))
                )
                .build();


        lineUpRow2 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET.mirror(), END_LINEUP_START_INTAKE2.mirror()))
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(-90)),
                        mirrorHeading(heading(180))
                )
                .build();

        intakeRow2 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE2.mirror(), END_INTAKE_START_SCORE2.mirror()))
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(180)),
                        mirrorHeading(heading(180))
                )
                .build();

        scoreRow2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                END_INTAKE_START_SCORE2.mirror(),
                                CONTROL1_SCORE_ROW2.mirror(),
                                UNIVERSAL_SCORE_TARGET.mirror()
                        )
                )
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(180)),
                        mirrorHeading(heading(-90))
                )
                .build();

        lineUpRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(UNIVERSAL_SCORE_TARGET.mirror(), END_LINEUP_START_INTAKE3.mirror()))
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(-90)),
                        mirrorHeading(heading(180))
                )
                .build();

        intakeRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(END_LINEUP_START_INTAKE3.mirror(), END_INTAKE_START_SCORE3.mirror()))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        END_INTAKE_START_SCORE3.mirror(),
                        UNIVERSAL_SCORE_TARGET.mirror()
                ))
                .setLinearHeadingInterpolation(
                        mirrorHeading(heading(180)),
                        mirrorHeading(heading(-90))
                )
                .build();
    }

    private double mirrorHeading(double heading) {
        return Math.PI - heading;
    }
}