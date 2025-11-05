package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class GlobalAutonomousPoses {

    public static double heading(double degrees) {
        return Math.toRadians(degrees);
    }

    @Configurable
    public static class GoalSidePoses {
        // The global scoring location
        public static Pose UNIVERSAL_SCORE_TARGET = new Pose(53.179, 93.882);
        // preload points
        public static Pose START_PRELOAD = new Pose(31.636, 136.515);

        // ROW 2
        public static Pose END_LINEUP_START_INTAKE = new Pose(48.190, 89.904);
        public static Pose END_INTAKE_START_SCORE = new Pose(12.360, 83.451);
        // score row 1 bezier control points
        public static Pose CONTROL1_SCORE_ROW1 = new Pose(52.726, 78.235);

        // ROW 2
        public static Pose END_LINEUP_START_INTAKE2 = new Pose(53.377, 60.020);
        public static Pose END_INTAKE_START_SCORE2 = new Pose(5, 64);
        public static Pose CONTROL1_SCORE_ROW2 = new Pose(74.965, 60.731);

        // ROW 3
        public static Pose END_LINEUP_START_INTAKE3 = new Pose(53.140, 36.534);
        public static Pose END_INTAKE_START_SCORE3 = new Pose(5.183, 39.059);
        public static Pose CONTROL1_SCORE_ROW3 =  new Pose(80.659, 31.789);

    }

    @Configurable
    public static class AudienceSidePoses {

    }
}
