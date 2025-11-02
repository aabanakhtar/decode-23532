package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

import java.util.Objects;

public class MecanumDrive extends SubsystemBase {
    public class AimAtTarget {
        public double distance;
        public double heading;

        public AimAtTarget(double distance, double heading) {
            this.distance = distance;
            this.heading = heading;
        }
    }

    public Follower follower;
    public static Pose lastPose = new Pose(0, 0, 0);
    public static Pose blueGoalPose = new Pose(13, 134);
    public static Pose redGoalPose = blueGoalPose.mirror();

    public MecanumDrive(HardwareMap map, Pose startingPose) {
        this.follower = Constants.createFollower(map);
        follower.setStartingPose(startingPose == null ? new Pose(0, 0, 0) : startingPose);
        follower.update();
    }

    @Override
    public void periodic() {
        lastPose = follower.getPose();
        DuneStrider robot = DuneStrider.get();
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

    public AimAtTarget getShooterPositionPinpointRel() {
        Pose targetPose;
        if (DuneStrider.alliance == DuneStrider.Alliance.BLUE) {
            targetPose = blueGoalPose;
        } else {
            targetPose = redGoalPose;
        }

        Pose currentPose = getPose();
        double targetAngle = Math.toDegrees(Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));
        double distance = Math.sqrt(Math.pow(targetPose.getX() - currentPose.getX(), 2) + Math.pow(targetPose.getY() - currentPose.getY(), 2));
        return new AimAtTarget(distance, targetAngle);
    }
}
