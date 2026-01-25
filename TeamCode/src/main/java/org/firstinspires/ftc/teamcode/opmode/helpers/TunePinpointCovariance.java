package org.firstinspires.ftc.teamcode.opmode.helpers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.Timing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Tune Pinpoint Covariances")
public class TunePinpointCovariance extends OpMode {
    private Motor shooterLeft;
    private Motor shooterRight;
    private ArrayList<Double> pinpointAxisReadings;
    private ArrayList<Double> pinpointThetaReadings;
    private Follower follower;

    public static long TUNING_TIME_SECONDS = 100;
    private int samplesCollected = 0;
    private final Timing.Timer stepTimer = new Timing.Timer(1, TimeUnit.SECONDS);

    double sigmaResultAxis = 0;
    double sigmaResultTheta = 0;
    boolean finished = false;

    @Override
    public void init() {
        pinpointAxisReadings = new ArrayList<>();
        pinpointThetaReadings = new ArrayList<>();

        shooterRight = new Motor(hardwareMap, "shooterRight");
        shooterLeft = new Motor(hardwareMap, "shooterLeft");
        shooterRight.setInverted(true);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        follower.startTeleopDrive();

        telemetry.addLine("Ready. Press START to begin 100s drift test.");
        telemetry.update();
    }

    @Override
    public void start() {
        shooterLeft.set(0.5);
        shooterRight.set(0.5);
        stepTimer.start();
    }

    @Override
    public void loop() {
        follower.update();

        if (samplesCollected < TUNING_TIME_SECONDS) {
            if (stepTimer.done()) {
                pinpointAxisReadings.add(follower.getPose().getX());
                pinpointThetaReadings.add(Math.toDegrees(follower.getPose().getHeading()));

                samplesCollected++;
                stepTimer.start();
            }

            telemetry.addData("Status", "COLLECTING DRIFT DATA");
            telemetry.addData("Progress", "%d / %d seconds", samplesCollected, TUNING_TIME_SECONDS);
        }
        else if (!finished) {
            sigmaResultAxis = calculateSigma(pinpointAxisReadings);
            sigmaResultTheta = calculateSigma(pinpointThetaReadings);

            shooterLeft.set(0);
            shooterRight.set(0);
            finished = true;
        }
        else {
            telemetry.addData("STATUS", "TUNING COMPLETE");
            telemetry.addData("Axis Sigma", sigmaResultAxis);
            telemetry.addData("Theta Sigma", sigmaResultTheta);
            telemetry.addData("Final Axis Q (Variance)", Math.pow(sigmaResultAxis, 2));
            telemetry.addData("Final Theta Q (Variance)", Math.pow(sigmaResultTheta, 2));
        }

        telemetry.update();
    }

    private double calculateSigma(ArrayList<Double> samples) {
        if (samples.size() < 2) return 0;

        double sampleInterval = 1.0; // 1 second between samples

        // Method 1: First to last
        double firstToLast = Math.abs(samples.get(samples.size() - 1) - samples.get(0));
        double totalTime = (samples.size() - 1) * sampleInterval;
        double sigma1 = firstToLast / Math.sqrt(totalTime);

        // Average the methods
        return (sigma1 + sigma1) / 2.0;
    }

}