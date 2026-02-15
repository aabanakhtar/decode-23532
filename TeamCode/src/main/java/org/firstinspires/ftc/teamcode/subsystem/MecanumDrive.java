package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.cmd.Commandlet.run;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.shooterTimeRegression;

import android.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;
import org.firstinspires.ftc.teamcode.utilities.SubsystemLooptimeAverager;

import java.util.Objects;

@Config
public class MecanumDrive extends SubsystemBase {
    public static double TURRET_OFFSET = -2.06;
    private SubsystemLooptimeAverager averager = new SubsystemLooptimeAverager();
    private DuneStrider robot = DuneStrider.get();

    public Command resetHeading() {
        return run(() -> robot.drive.follower.setHeading(0));
    }

    public static class AimAtTarget {
        public double distance;
        public double heading;

        public AimAtTarget(double distance, double heading) {
            this.distance = distance;
            this.heading = heading;
        }
    }

    public AimAtTarget lastAimTarget = new AimAtTarget(0, 0);

    public Follower follower;
    public static Pose lastPose = new Pose(0, 0, 0);
    public static final Pose blueGoalPose = new Pose(0, 144);
    public static final Pose redGoalPose = blueGoalPose.mirror();

    public MecanumDrive(HardwareMap map, Pose startingPose) {
        this.follower = Constants.createFollower(map);
        follower.setStartingPose(startingPose == null ? new Pose(0, 0, 0) : startingPose);
        follower.update();
    }

    @Override
    public void periodic() {
        averager.mark();
        lastPose = follower.getPose();

        lastAimTarget = getShooterPositionPinpointRel2();
        robot.flightRecorder.addLine("======DRIVETRAIN:=======");
        robot.flightRecorder.addData("goal heading", lastAimTarget.heading);
        robot.flightRecorder.addData("X:", lastPose.getX());
        robot.flightRecorder.addData("Y:", lastPose.getY());
        robot.flightRecorder.addData("Heading", Math.toDegrees(lastPose.getHeading()));
        robot.flightRecorder.addData("avg ms", averager.getAvgMs());

        follower.update();

        averager.endMark();
    }

    public void setTeleOpDrive(double forward, double strafe, double rotation) {
        follower.setTeleOpDrive(forward, strafe, rotation, false);
    }

    /**
     * Get the current estimated robot pose
     */
    public Pose getPose() {
        return follower.getPose();
    }

    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public AimAtTarget getAimTarget() {
        return lastAimTarget;
    }

    private AimAtTarget getShooterPositionPinpointRel2() {
        Pose chosenPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? blueGoalPose : redGoalPose;
        Pose currPose = getPose();

        Pose aimAtPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? chosenPose.plus(new Pose(0, 0, 0)) : chosenPose.minus(new Pose(0, 0, 0));

        double distance = chosenPose.distanceFrom(currPose) / 12.0;
        // Math.PI makes it face the other direction.
        double turretXOffset = TURRET_OFFSET * Math.cos(currPose.getHeading());
        double turretYOffset = TURRET_OFFSET * Math.sin(currPose.getHeading());

        double absAngleToTarget = Math.atan2(
                aimAtPose.getY() - (currPose.getY() + turretYOffset),
                aimAtPose.getX() - (currPose.getX() + turretXOffset)
        );

        double robotHeading = currPose.getHeading(); // rad
        double turretRelativeAngleRad = AngleUnit.normalizeRadians(absAngleToTarget - robotHeading);
        double turretRelativeAngleDeg = Math.toDegrees(turretRelativeAngleRad);

        // clamp and return
        return new AimAtTarget(distance, turretRelativeAngleDeg);
    }
}