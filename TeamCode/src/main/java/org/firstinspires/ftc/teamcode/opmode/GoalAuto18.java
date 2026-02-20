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
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.END_INTAKE_START_SCORE2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_CONTROL_POINT2;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.START_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.GoalSidePoses.UNIVERSAL_SCORE_TARGET;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.heading;
import static org.firstinspires.ftc.teamcode.opmode.helpers.GlobalAutonomousPoses.mirrorHeading;

import com.acmerobotics.dashboard.config.Config;
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
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous: 18 Artifact Gate Cycling Configurable", group = "auto", preselectTeleOp = "TeleOp")
public class GoalAuto18 extends OpMode {
    // Mechanical
    public static double SHOOTER_TRANSFER_DELAY = 720.0;
    public static double INTAKE_RECOLLECTION_TIMEOUT = 300.0;
    public static long INTAKE_STOP_DELAY = 0;
    public static double PRELOAD_MAX_SPEED = 0.7;

    // Gate
    public static long GATE_DURATION = 700;
    public static double GATE_HEADING = 160;
    public static double GATE_CYCLE_TM = 4000;
    public static double GATE_CYCLE_PWSCALE_START = 0.55;

    // paths
    public static double PW_SCALE_GATE_CYCLE_SPEED = 0.15;
    public static double ROW2_INTAKE_PATH_SPEED = 0.8;

    // global path stuff
    public static double PW_SCALE_BRAKE_THRESHOLD = 0.7;
    public static double PW_SCALE_PATH_SPEED = 0.15;
    public static double PRELOAD_SLOWDOWN_THRESH = 0.9;
    public static double SCHEDULE_SHOT_PRE = 0.3;
    public static double BRAKE_THRESHOLD_SHOTS = 0.67;

    private DuneStrider robot;
    private PathChain shootPreload;
    private PathChain intakeRow1, intakeRow2;
    private PathChain scoreRow1, scoreRow2;
    private PathChain gateCycle, shootGate;
    private PathChain parkRP;

    public static int nRows = 4;

    @Override
    public void init() {
        Pose startPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? START_PRELOAD.setHeading(heading(90)) : START_PRELOAD.mirror().setHeading(heading(90));

        robot = DuneStrider.get().init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);
        robot.eyes.setEnabled(false);
        Turret.offset_angle = 0;
        Follower follower = robot.drive.follower;
        buildPathChains(follower);

        // we ball
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        execPreloadAndR1(),
                        execRowGate(),
                        execRowGate(),
                        execRowGate(),
                        execRow1(),
                        go(robot.drive.follower, parkRP, 1),
                        run(() -> robot.shooter.setVelocity(0)),
                        run(this::completeShot)
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
                new FollowPathCommand(robot.drive.follower, shootPreload, true),
                shoot((long)SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRowGate() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, gateCycle, 1.0)
                        .raceWith(waitFor((long)GATE_CYCLE_TM)),
                waitFor(GATE_DURATION),

                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, shootGate, true, 1.0)
                        )
                ),

                // go home and score
                shoot((long) SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRow1() {
        return new SequentialCommandGroup(
                // RUN INTAKE WITH ALIGN
                run(() -> robot.intake.closeLatch()),
                // eat the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, intakeRow1, false, 1.0),

                // let the intake regen
                fork (
                        new SequentialCommandGroup(
                                waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, scoreRow1, true, 1.0)
                        )
                ),

                // go home and score
                shoot((long) SHOOTER_TRANSFER_DELAY)
        );
    }

    private Command execRow2() {
        return new SequentialCommandGroup(
                run(() -> robot.intake.closeLatch()),
                // turn on the intake and eat up the balls
                intakeSet(Intake.Mode.INGEST),
                new FollowPathCommand(robot.drive.follower, intakeRow2, true, 1),
                waitFor(INTAKE_STOP_DELAY),

                fork(
                        new SequentialCommandGroup(
                                waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                intakeSet(Intake.Mode.OFF)
                        ),
                        new SequentialCommandGroup(
                                run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                new FollowPathCommand(robot.drive.follower, scoreRow2, true, 1)
                        )
                ),

                // go home and score
                shoot((long) SHOOTER_TRANSFER_DELAY)
        );
    }

    private void prepareShot() {
        robot.shooter.setMode(Shooter.Mode.DYNAMIC);
        robot.turret.setMode(Turret.Mode.PINPOINT);
        robot.intake.closeLatch();
    }

    private void takeShot() {
        Intake.INGEST_MOTOR_SPEED = 0.75;
        robot.intake.openLatch();
        robot.intake.setMode(Intake.Mode.INGEST);
    }

    private void completeShot() {
        Intake.INGEST_MOTOR_SPEED = 1;
    }

    private void prepareIntake() {
        robot.intake.closeLatch();
        robot.intake.setMode(Intake.Mode.INGEST);
    }

    private void disableIntake() {
        robot.intake.setMode(Intake.Mode.OFF);
    }

    private void buildPathChains(Follower follower) {
        shootPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                            mPBA(START_PRELOAD),
                            mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPowerScaling(PRELOAD_MAX_SPEED);
                    prepareShot();
                })
                .addParametricCallback(SCHEDULE_SHOT_PRE, this::takeShot)
                .addParametricCallback(PRELOAD_SLOWDOWN_THRESH, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setConstantHeadingInterpolation(heading(90))


                // get row 2
                .addPath(
                        new BezierCurve(
                                mPBA(UNIVERSAL_SCORE_TARGET),
                                mPBA(INTAKE_CONTROL_POINT2),
                                mPBA(END_INTAKE_START_SCORE2)
                        )
                )
                .addParametricCallback(0, () -> {
                    completeShot();
                    prepareIntake();
                })
                .addParametricCallback(0.4, () -> follower.setMaxPowerScaling(ROW2_INTAKE_PATH_SPEED))
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))

                // shoot row2
                .addPath(
                        new BezierCurve(
                                mPBA(END_INTAKE_START_SCORE2),
                                mPBA(new Pose(28, 59)),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(0, this::prepareShot)
                .addParametricCallback(0.2, this::disableIntake)
                .addParametricCallback(BRAKE_THRESHOLD_SHOTS, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setReversed()
                .build();

        intakeRow1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                            mPBA(UNIVERSAL_SCORE_TARGET),
                            mPBA(END_INTAKE_START_SCORE)
                        )
                )
                .addParametricCallback(0, () -> follower.setMaxPowerScaling(ROW2_INTAKE_PATH_SPEED - 0.1))
                .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setConstantHeadingInterpolation(mHBA(heading(180 - 10)))
                .build();

        scoreRow1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                mPBA(END_INTAKE_START_SCORE),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setConstantHeadingInterpolation(mHBA(heading(180)))
                .addParametricCallback(0.65, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .build();

        gateCycle = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(UNIVERSAL_SCORE_TARGET),
                                mPBA(new Pose(45, 56)),
                                mPBA(END_GATE)
                        )
                )
                .addParametricCallback(GATE_CYCLE_PWSCALE_START, () -> follower.setMaxPowerScaling(PW_SCALE_GATE_CYCLE_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setLinearHeadingInterpolation(mHBA(heading(180)), mHBA(heading(GATE_HEADING)))
                .setTimeoutConstraint(100)
                .setTValueConstraint(0.95)
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                mPBA(END_GATE),
                                mPBA(new Pose(34, 56)),
                                mPBA(UNIVERSAL_SCORE_TARGET)
                        )
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(BRAKE_THRESHOLD_SHOTS, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                .setTValueConstraint(0.97)
                .setReversed()
                .build();

        parkRP = follower.pathBuilder()
                .addPath(new BezierLine(mPBA(UNIVERSAL_SCORE_TARGET), mPBA(new Pose(48, 72))))
                .setTangentHeadingInterpolation()
                .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
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