package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.Nullable;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.DuneStrider;

import java.util.List;

public class HugeEyes extends SubsystemBase {
    private final Limelight3A limelight;
    private final DuneStrider robot;

    public static boolean disabled = true;
    public static double MAX_SOURCE_DISPARITY = 3.0; // in
    public static double MAX_RELOCALIZE_VELOCITY = 5.0; // in / sec


    public HugeEyes() {
        robot = DuneStrider.get();
        limelight = robot.limelight;
    }

    public void setEnabled(boolean enabled) {
        this.disabled = !enabled;
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            Position position = botPose.getPosition();

            double x = DistanceUnit.INCH.fromMeters(position.x);
            double y = DistanceUnit.INCH.fromMeters(position.y);
            double heading = Math.toRadians(botPose.getOrientation().getYaw());
            robot.flightRecorder.addData("heading", Math.toDegrees(heading));

            Pose pedroPose = new Pose(y + 72, 72 - x, heading - Math.PI/2);
            robot.flightRecorder.addData("Bot pose (Pedro)", pedroPose.toString());
            robot.flightRecorder.addData("Is valid pose", verifyLimelightPose(pedroPose, robot.drive.getPose()));

            if (verifyLimelightPose(pedroPose, robot.drive.getPose()) && !disabled) {
                robot.drive.follower.setPose(pedroPose);
            }
        }

    }

    public boolean verifyLimelightPose(Pose newEstimate, Pose current) {
        if (Math.abs(current.distanceFrom(newEstimate)) < MAX_SOURCE_DISPARITY) {
            return false;
        }

        if (!(newEstimate.getX() < 144 && newEstimate.getY() < 144 && newEstimate.getX() > 0 && newEstimate.getY() > 0)) {
            return false;
        }

        return !(Math.abs(robot.drive.getVelocity().getMagnitude()) > MAX_RELOCALIZE_VELOCITY);
    }
}
