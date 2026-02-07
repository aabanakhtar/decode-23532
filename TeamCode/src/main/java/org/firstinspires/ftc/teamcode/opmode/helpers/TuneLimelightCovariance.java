package org.firstinspires.ftc.teamcode.opmode.helpers;

import androidx.appcompat.view.menu.MenuBuilder;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Tune Limelight Covariances")
public class TuneLimelightCovariance extends OpMode {
    Motor shooterLeft;
    Motor shooterRight;
    ArrayList<Double> limelightAxisReadings;
    ArrayList<Double> limelightThetaReadings;
    Follower follower;
    Limelight3A limelight3A;
    Timing.Timer timer = new Timing.Timer(10, TimeUnit.SECONDS);

    double covarianceResultAxis = 0;
    double covarianceResultTheta = 0;

    boolean finished = false;

    @Override
    public void init() {
        limelightAxisReadings = new ArrayList<>();
        limelightThetaReadings = new ArrayList<>();

        telemetry.addLine("Ready to tune. OpMode will spin up shooters and measure limelight variances for 10 seconds. ENSURE ROBOT IS AT 0 DEGREES");
        telemetry.update();

        shooterRight = new Motor(hardwareMap, "shooterRight");
        shooterLeft = new Motor(hardwareMap, "shooterLeft");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);

        follower.setPose(new Pose(72, 72, 0));
        follower.startTeleopDrive();
        limelight3A.pipelineSwitch(2);
        shooterRight.setInverted(true);
    }

    @Override
    public void start() {
        shooterLeft.set(0.5);
        shooterRight.set(0.5);
        timer.start();
        limelight3A.start();
    }

    public static double getCovariance(ArrayList<Double> samples) {
        if (samples.size() < 2) {
            throw new IllegalArgumentException("Could not collect enough data to measure!");
        }

        double mean = 0;
        for (double i : samples) {
            mean += i;
        }

        mean = mean / samples.size();

        double variance = 0;
        for (double i : samples) {
            variance += Math.pow(i - mean, 2);
        }

        variance = variance / (samples.size() - 1);
        return variance;
    }

    private void collectSamples() {
        telemetry.addLine("Collecting data.....");

        limelight3A.updateRobotOrientation(Math.toDegrees(follower.getPose().getHeading() + Math.PI / 2));
        LLResult result = limelight3A.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose();
            Position position = botPose.getPosition();

            double y = DistanceUnit.INCH.fromMeters(position.y);
            double heading = Math.toRadians(botPose.getOrientation().getYaw());

            Pose pedroPose = new Pose(y + 72, 0, heading - Math.PI/2);

            limelightAxisReadings.add(pedroPose.getX());
            limelightThetaReadings.add(Math.toDegrees(pedroPose.getHeading()));
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        if (!timer.done()) {
            follower.update();
            collectSamples();
            telemetry.addData("STATUS", "Collecting Data ... Remaining Time: " + timer.remainingTime());
        }
        else if (!finished) {
            shooterLeft.set(0);
            shooterRight.set(0);

            covarianceResultAxis = getCovariance(limelightAxisReadings);
            covarianceResultTheta = getCovariance(limelightThetaReadings);

            finished = true;
        } else {
            telemetry.addData("STATUS", "DONE");
            telemetry.addData("Samples Collected (should be >300)", limelightAxisReadings.size());
            telemetry.addData("Axis Covariance (in^2)", covarianceResultAxis);
            telemetry.addData("Heading Covariance (deg^2)", covarianceResultTheta);
        }

        telemetry.update();
    }
}
