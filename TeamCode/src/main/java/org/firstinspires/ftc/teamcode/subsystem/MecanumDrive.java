package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Shooter.shooterTimeRegression;

import android.util.Range;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

import java.util.Objects;

@Config
public class MecanumDrive extends SubsystemBase {
    private static final double TURRET_OFFSET = 2.0599;
    public static double PREDICT_FACTOR = 0.35;
    DuneStrider robot = DuneStrider.get();

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
        lastPose = follower.getPose();

        lastAimTarget = getShooterPositionPinpointRel2();
        robot.flightRecorder.addLine("======DRIVETRAIN:=======");
        robot.flightRecorder.addData("goal heading", lastAimTarget.heading);
        robot.flightRecorder.addData("goal distance", lastAimTarget.distance);

        robot.flightRecorder.addData("X:", lastPose.getX());
        robot.flightRecorder.addData("Y:", lastPose.getY());
        robot.flightRecorder.addData("Heading", Math.toDegrees(lastPose.getHeading()));

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

    public double getRadialVelocityToGoal() {
        return 0;
    }

    public double getTangentVelocityToGoal() {
        Vector robotPosition = getPose().getAsVector();
        Vector goalPosition = (DuneStrider.alliance == DuneStrider.Alliance.BLUE ? blueGoalPose : redGoalPose).getAsVector();

        // Calculate vector from robot to goal
        Vector toGoal = goalPosition.minus(robotPosition);

        // Calculate angle to goal (field-relative)
        double angleToGoalField = Math.atan2(toGoal.getYComponent(), toGoal.getXComponent());

        // Get robot velocity scalar
        double robotSpeed = getVelocity().getMagnitude();

        if (robotSpeed < 3) {
            return 0;
        }

        double robotVelocityAngle = Math.atan2(getVelocity().getYComponent(), getVelocity().getXComponent());

        // Calculate tangential component
        double deltaTheta = AngleUnit.normalizeRadians(robotVelocityAngle - angleToGoalField);
        return robotSpeed * Math.sin(deltaTheta);
    }

    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }

    public Vector getAcceleration() {
        return follower.getAcceleration();
    }

    public AimAtTarget getAimTarget() {
        return lastAimTarget;
    }

    private AimAtTarget getShooterPositionPinpointRel2() {
        Pose chosenPose = DuneStrider.alliance == DuneStrider.Alliance.BLUE ? blueGoalPose : redGoalPose;
        Pose currPose = getPose();

        // aim logic to help prevent undershoot on the edge of the top tiles
        Pose aimAtPose;
        if (currPose.getY() > 72.0 && DuneStrider.alliance == DuneStrider.Alliance.BLUE) {
            aimAtPose = new Pose(7, 144 - 7);
        } else if (currPose.getY() > 72.0 && DuneStrider.alliance == DuneStrider.Alliance.RED) {
            aimAtPose = new Pose(144 - 7, 144 - 7);
        } else {
            aimAtPose = chosenPose;
        }

        double distance = chosenPose.distanceFrom(chosenPose) / 12.0;
        double turretXOffset = -TURRET_OFFSET * Math.cos(currPose.getHeading());
        double turretYOffset = -TURRET_OFFSET * Math.sin(currPose.getHeading());

        double absAngleToTarget = Math.atan2(
                aimAtPose.getY() - (currPose.getY() + turretYOffset),
                aimAtPose.getX() - (currPose.getX() + turretXOffset)
        );

        double robotHeading = currPose.getHeading(); // rad
        double turretRelativeAngleRad = absAngleToTarget - (robotHeading);

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