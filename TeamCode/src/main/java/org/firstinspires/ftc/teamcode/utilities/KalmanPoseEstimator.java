package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.robot.DuneStrider;

public class KalmanPoseEstimator {
    public double drift;
    public double uncertainty;
    public double PINPOINT_VARIANCE;
    public double LIMELIGHT_VARIANCE;

    public KalmanPoseEstimator(double initialDrift, double initialUncertainty, double Q, double R) {
        this.drift = initialDrift;
        this.uncertainty = initialUncertainty;
        this.PINPOINT_VARIANCE = Q;
        this.LIMELIGHT_VARIANCE = R;
    }

    public void updateKalmanUncertainty(double dt) {
        uncertainty = uncertainty + dt * PINPOINT_VARIANCE;

        double kalmanGain = uncertainty / (uncertainty + LIMELIGHT_VARIANCE);
        DuneStrider.get().flightRecorder.addData("KALMAN GAIN", kalmanGain);
    }

    public double getDriftKalman(double pinpointReading, double limelightReading) {
        double Z = pinpointReading - limelightReading;
        double kalmanGain = uncertainty / (uncertainty + LIMELIGHT_VARIANCE);

        drift = drift + kalmanGain * (Z - drift);
        uncertainty = (1 - kalmanGain) * uncertainty;

        DuneStrider.get().flightRecorder.addData("KALMAN GAIN", kalmanGain);
        return drift;
    }

}