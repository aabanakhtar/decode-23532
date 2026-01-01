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
        public static Pose START_PRELOAD = new Pose(31.636, 136.515);

        // ROW 1
        public static Pose INTAKE_CONTROL_POINT = new Pose(55, 85);
        public static Pose END_INTAKE_START_SCORE = new Pose(19, 84.5);

        // ROW 2
        public static Pose END_INTAKE_START_SCORE2 = new Pose(14.5, 68.5);
        public static Pose INTAKE_CONTROL_POINT2 = new Pose(60, 61);
        public static Pose INTAKE_CONTROL_SCORE_R2 = new Pose(58, 73);

        // JIAR Sync Auto related
        public static Pose OPEN_GATE_OPEN = new Pose(20, 69);

        // ROW 3
        public static Pose INTAKE_CONTROL_POINT3 = new Pose(75, 36);
        public static Pose SCORE_CONTROL_POINT3 = new Pose(54, 46);
        public static Pose END_INTAKE_START_SCORE3 = new Pose(7.5, 38.059);

        // ROW 4
        public static Pose INTAKE_CONTROL_POINT4 = new Pose(41, 10);
        public static Pose INTAKE_CONTROL_POINT4_0 = new Pose(8,  63);
        public static Pose END_INTAKE_START_SCORE_HP_ZONE = new Pose(6, 15);
    }

    @Configurable
    public static class AudienceSidePoses {
        public static Pose AUNIVERSAL_SCORE_TARGET = new Pose(63, 14);

        // row 3
        public static Pose ACONTROL1_LINEUP_ROW3 = new Pose(68.203, 35.635);
        public static Pose ALINEUP_ROW3_END = new Pose(39.140, 35.635);

        public static Pose AEND_INTAKE_ROW3 = new Pose(15, 35);
        // score row 3
        public static Pose ACONTROL1_SCORE_ROW3 = new Pose(66,  39);
    }
}
