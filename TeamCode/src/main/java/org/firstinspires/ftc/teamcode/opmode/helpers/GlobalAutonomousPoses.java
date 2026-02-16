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


    public static double mirrorHeading(double heading) {
        return Math.PI - heading;
    }

    @Configurable
    public static class GoalSidePoses {
        // The global scoring location
        public static Pose UNIVERSAL_SCORE_TARGET = new Pose(56, 90);
        // preload points
        public static Pose START_PRELOAD = new Pose(30, 134);

        // ROW 1
        public static Pose INTAKE_CONTROL_POINT = new Pose(55, 85);
        public static Pose END_INTAKE_START_SCORE = new Pose(15, 79);

        // ROW 2
        public static Pose END_INTAKE_START_SCORE2 = new Pose(15, 59.5);
        public static Pose INTAKE_CONTROL_POINT2 = new Pose(62, 59);
        public static Pose INTAKE_CONTROL_SCORE_R2 = new Pose(28, 55);

        public static Pose INTAKE_GATE = new Pose(30, 61.5);
        public static Pose END_GATE = new Pose (11, 61);
    }

    @Configurable
    public static class AudienceSidePoses {
        public static Pose AUNIVERSAL_SCORE_TARGET = new Pose(63, 20);

        // row 3
        public static Pose ACONTROL1_LINEUP_ROW3 = new Pose(68.203, 35.635);
        public static Pose ALINEUP_ROW3_END = new Pose(39.140, 35.635);

        public static Pose AEND_INTAKE_ROW3 = new Pose(15, 35);
        // score row 3
        public static Pose ACONTROL1_SCORE_ROW3 = new Pose(66,  39);
    }
}
