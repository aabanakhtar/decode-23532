package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.If;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.fork;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.go;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.intakeSet;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.nothing;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.shoot;
import static org.firstinspires.ftc.teamcode.cmd.Commandlet.waitFor;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_GATE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_GATE_RED;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.START_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.UNIVERSAL_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.heading;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.mirrorHeading;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.Mode.OFF;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous: 15 CLOSE", group = "auto", preselectTeleOp = "TeleOp")
public class GoalAuto18 extends OpMode {
    public static Pose ROW2_INTAKE_POSE = new Pose(5, 59);
    public static  Pose ROW1_INTAKE_POSE = new Pose(13, 83);
    public static Pose GATE_INTAKE_POSE = new Pose(7, 61);
    public static Pose GATE_OFFSET_POSE = new Pose(7, 54);
    public static Pose END_INTAKE_START_SCORE3 = new Pose(7, 36);
    public static Pose BOOM_GATE = new Pose(12, 64);

    // Mechanical
    public static double SHOOTER_TRANSFER_DELAY = 750;
    public static double INTAKE_RECOLLECTION_TIMEOUT = 300.0;
    public static long INTAKE_STOP_DELAY = 0;

    // Gate
    public static long GATE_DURATION = 900;
    public static double GATE_HEADING = 169;
    public static double OFFSET_HEADING = 130;
    public static double GATE_CYCLE_TM = 4000;

    private DuneStrider robot;
    private PathChain shootPreload;
    private PathChain intakeRow1, intakeRow2, intakeGate, intakeRow3, offsetGate;
    private PathChain scoreRow1, scoreRow2, scoreGate, scoreRow3, gateSmash;
    private PathChain gateCycle, shootGate;
    private PathChain parkRP;

    public static int nRows = 4;

    @Override
    public void init() {
        Pose startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? START_PRELOAD.setHeading(heading(90)) : START_PRELOAD.mirror().setHeading(heading(90));

        robot = DuneStrider.get().init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);
        //robot.eyes.setEnabled(false);
        Turret.offset_angle = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? 2 : 0;
        Follower follower = robot.drive.follower;
        buildPathChains(follower);

        // we ball
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        execPreloadAndR1(),
                        execRow2(),
                        execRowGate(),
                        execRow1(),
                        execRowGate(),
                        go(follower, parkRP, 1)
                )
        );
    }

    @Override
    public void init_loop() {
        robot.turret.loadAngle(robot.analogEncoder.getCurrentPosition());
        telemetry.addLine("====DUNESTRIDER PRE-MATCH Config=====");
        telemetry.addData("|| > ALLIANCE:", DuneStrider.alliance.toString());
        telemetry.addData("|| > ROWS:", nRows);
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.endLoop();
    }

    private Command execPreloadAndR1() {
        return new SequentialCommandGroup(
                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                new FollowPathCommand(robot.drive.follower, shootPreload, true),
                waitFor(300),
                shoot((long)SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRow2() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                run(() -> robot.intake.setMode(Intake.Mode.INGEST)),

                new FollowPathCommand(robot.drive.follower, intakeRow2, 1),
                //new FollowPathCommand(robot.drive.follower, gateSmash, 1),
                //waitFor(500),
                new FollowPathCommand(robot.drive.follower, scoreRow2, 1),
                waitFor(100),
                shoot((long)SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRow3() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                run(() -> robot.intake.setMode(Intake.Mode.INGEST)),

                new FollowPathCommand(robot.drive.follower, intakeRow3, 1),

                new FollowPathCommand(robot.drive.follower, scoreRow3, 1),
                waitFor(400),
                shoot((long)SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRow1() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                run(() -> robot.intake.setMode(Intake.Mode.INGEST)),

                new FollowPathCommand(robot.drive.follower, intakeRow1, 1),

                new FollowPathCommand(robot.drive.follower, scoreRow1, 1),
                waitFor(100),
                shoot((long)SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRowGate() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                run(() -> robot.intake.setMode(Intake.Mode.INGEST)),

                new FollowPathCommand(robot.drive.follower, intakeGate, 1),
                waitFor(200),
                new FollowPathCommand(robot.drive.follower, offsetGate, 1),
                waitFor(GATE_DURATION),
                new FollowPathCommand(robot.drive.follower, scoreGate, 1),
                waitFor(100),
                shoot((long)SHOOTER_TRANSFER_DELAY)
        );
    }

    private void buildPathChains(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                            mPBA(START_PRELOAD),
                            mPBA(new Pose(36, 105)),
                            mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeRow2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(UNIVERSAL_SCORE_TARGET),
                                mPBA(new Pose(49, 52)),
                                mPBA(ROW2_INTAKE_POSE)
                        )
                )
                .addParametricCallback(0.5, () -> follower.setMaxPowerScaling(0.8))
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .addParametricCallback(0.9, () -> follower.setMaxPowerScaling(1))
                .build();

        gateSmash = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(ROW2_INTAKE_POSE),
                                mPBA(new Pose(36, 58)),
                                mPBA(BOOM_GATE)
                        )
                )
                .addParametricCallback(0, () -> follower.setMaxPowerScaling(0.7))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1))
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .build();

        intakeRow1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(UNIVERSAL_SCORE_TARGET),
                                mPBA(ROW1_INTAKE_POSE)
                        )
                )
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .build();

        scoreRow2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                //mPBA(BOOM_GATE),
                                mPBA(END_INTAKE_START_SCORE2),
                                mPBA(new Pose(53, 68)),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .addParametricCallback(0.6, () -> robot.intake.setMode(OFF))
                .addParametricCallback(0.5, () -> robot.shooter.setMode(Shooter.Mode.DYNAMIC))
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(GATE_OFFSET_POSE),
                                mPBA(new Pose(53, 62)),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .addParametricCallback(0.05, () -> robot.intake.setMode(OFF))
                .addParametricCallback(0.5, () -> robot.shooter.setMode(Shooter.Mode.DYNAMIC))
                .build();

        scoreRow1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(ROW1_INTAKE_POSE),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .addParametricCallback(0.7, () -> robot.intake.setMode(OFF))
                .addParametricCallback(0.5, () -> robot.shooter.setMode(Shooter.Mode.DYNAMIC))
                .build();

        intakeGate = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        mPBA(UNIVERSAL_SCORE_TARGET),
                        mPBA(new Pose(53, 62)),
                        mPBA(GATE_INTAKE_POSE)
                    )
                )
                .setTValueConstraint(1)
                .setConstantHeadingInterpolation(mHBA(heading(GATE_HEADING)))
                .build();

       offsetGate = follower.pathBuilder()
               .addPath(
                    new BezierLine(
                            mPBA(GATE_INTAKE_POSE),
                            mPBA(GATE_OFFSET_POSE)
                    )
                )
                .setConstantHeadingInterpolation(mHBA(heading(OFFSET_HEADING)))
               .setTValueConstraint(0.8)
                .build();

        parkRP = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(UNIVERSAL_SCORE_TARGET),
                                mPBA(new Pose(52, 66))
                        )
                )
                .setLinearHeadingInterpolation(mHBA(heading(180)), mHBA(heading(135)))
                .build();

        intakeRow3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(UNIVERSAL_SCORE_TARGET),
                                mPBA(new Pose(66, 27)),
                                mPBA(new Pose(45, 44)),
                                mPBA(END_INTAKE_START_SCORE3)
                        )
                )
                .addParametricCallback(0.5, () -> follower.setMaxPowerScaling(0.8))
                .addParametricCallback(0.9, () -> follower.setMaxPowerScaling(1))
                .setTangentHeadingInterpolation()
                .build();

        scoreRow3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(END_INTAKE_START_SCORE3),
                                mPBA(new Pose(35, 83)),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.5, () -> robot.shooter.setMode(Shooter.Mode.DYNAMIC))
                .setReversed()
                .build();
    }

    // Mirror Pose based on alliance
    public static Pose mPBA(Pose poseToMirror) {
        if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
            return poseToMirror.mirror();
        } else {
            return poseToMirror;
        }
    }

    public static double mHBA(double heading) {
        if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
            return mirrorHeading(heading);
        } else {
            return heading;
        }
    }
}