package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Range;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

import java.util.Objects;

public class MecanumDrive extends SubsystemBase {
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
    public static final Pose blueGoalPose = new Pose(13, 134);
    public static final Pose redGoalPose = blueGoalPose.mirror();

    public MecanumDrive(HardwareMap map, Pose startingPose) {
        this.follower = Constants.createFollower(map);
        follower.setStartingPose(startingPose == null ? new Pose(0, 0, 0) : startingPose);
        follower.update();
    }

    @Override
    public void periodic() {
        lastPose = follower.getPose();
        DuneStrider robot = DuneStrider.get();

        lastAimTarget = getShooterPositionPinpointRel2();
        robot.telemetry.addData("goal heading", lastAimTarget.heading);
        robot.telemetry.addData("goal distance", lastAimTarget.distance);

        robot.telemetry.addData("X:", lastPose.getX());
        robot.telemetry.addData("Y:", lastPose.getY());
        robot.telemetry.addData("Heading", Math.toDegrees(lastPose.getHeading()));
        follower.update();
    }

    public void resetHeading(double newHeading) {
        follower.setPose(follower.getPose().setHeading(newHeading));
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

    /**
     * Get the current robot velocity
     */
    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public AimAtTarget getAimTarget() {
        return lastAimTarget;
    }

    private AimAtTarget getShooterPositionPinpointRel() {
        double distance = blueGoalPose.distanceFrom(follower.getPose()) / 12.0;
        double angle = Math.atan2(
            blueGoalPose.getY() - getPose().getY(), blueGoalPose.getX() - getPose().getX()
        );

        return new AimAtTarget(distance, Math.toDegrees(angle));
    }


    private AimAtTarget getShooterPositionPinpointRel2() {
        Pose chosenPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? blueGoalPose : redGoalPose;
        double distance = chosenPose.distanceFrom(follower.getPose()) / 12.0;
        double turretOffset = 4.5;
        double turretXOffset = turretOffset * Math.cos(getPose().getHeading() + Math.PI);
        double turretYOffset = turretOffset * Math.cos(getPose().getHeading() + Math.PI);

        double absAngleToTarget = Math.atan2(
                chosenPose.getY() - (getPose().getY() + turretYOffset),
                chosenPose.getX() - (getPose().getX() + turretXOffset)
        );

        double robotHeading = getPose().getHeading(); // rad
        double turretRelativeAngleRad = absAngleToTarget - (robotHeading + Math.PI);

        // do normalization
        while (turretRelativeAngleRad <= -Math.PI) {
            turretRelativeAngleRad += 2 * Math.PI;
        }
        while (turretRelativeAngleRad > Math.PI) {
            turretRelativeAngleRad -= 2 * Math.PI;
        }

        double turretRelativeAngleDeg = Math.toDegrees(turretRelativeAngleRad);

        double minAngle = -Turret.TURRET_MAX_ANGLE;
        double maxAngle = Turret.TURRET_MAX_ANGLE;
        double constrainedAngleDeg = Math.max(minAngle, Math.min(maxAngle, turretRelativeAngleDeg));
        return new AimAtTarget(distance, constrainedAngleDeg);
    }

}
