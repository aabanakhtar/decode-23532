package org.firstinspires.ftc.teamcode.opmode.helpers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class GlobalAutonomousPoses {

    public static double heading(double degrees) {
        return Math.toRadians(degrees);
    }

    public static Pose RED_RELOCALIZE = new Pose(7, 7.83, 0);
    public static Pose BLUE_RELOCALIZE = RED_RELOCALIZE.mirror().withHeading(heading(180));


    @Configurable
    public static class GoalSidePoses {
        // The global scoring location
        public static Pose UNIVERSAL_SCORE_TARGET = new Pose(56, 90);
        // preload points
        public static Pose START_PRELOAD = new Pose(31.636, 136.515 - 0.833);

        // ROW 2
        public static Pose END_LINEUP_START_INTAKE = new Pose(48.190, 89.904);
        public static Pose END_INTAKE_START_SCORE = new Pose(20, 83.451);
        // ROW 2
        public static Pose END_LINEUP_START_INTAKE2 = new Pose(49, 60.020);
        public static Pose END_INTAKE_START_SCORE2 = new Pose(11, 64);
        public static Pose CONTROL1_SCORE_ROW2 = new Pose(74.965, 60.731);

        // JIAR Sync Auto related
        public static Pose OPEN_GATE_START = new Pose(8, 59.455);
        public static Pose OPEN_GATE_SWING = new Pose(24, 59);
        public static Pose OPEN_GATE_CTRL = new Pose(30, 67);
        public static Pose OPEN_GATE_OPEN = new Pose(17, 69);

        // ROW 3
        public static Pose END_LINEUP_START_INTAKE3 = new Pose(49, 36.534);
        public static Pose END_INTAKE_START_SCORE3 = new Pose(11, 39.059);

        // ROW 4 comin soon
        public static Pose END_LINEUP_START_INTAKE_HP_ZONE = new Pose(9, 36);
        public static Pose END_INTAKE_START_SCORE_HP_ZONE = new Pose(9, 11);
    }

    @Configurable
    public static class AudienceSidePoses {
        public static Pose AUNIVERSAL_SCORE_TARGET = new Pose(63, 14);

        // row 3
        public static Pose ACONTROL1_LINEUP_ROW3 = new Pose(68.203, 35.635);
        public static Pose ALINEUP_ROW3_END = new Pose(39.140, 35.635);

        public static Pose AEND_INTAKE_ROW3 = new Pose(5, 35);
        // score row 3
        public static Pose ACONTROL1_SCORE_ROW3 = new Pose(66,  39);
    }
}
