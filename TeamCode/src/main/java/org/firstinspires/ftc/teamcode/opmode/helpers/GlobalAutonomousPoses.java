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
        public static Pose UNIVERSAL_SCORE_TARGET = new Pose(50, 79);
        // preload points
        public static Pose START_PRELOAD = new Pose(30, 134);

        // ROW 1
        public static Pose INTAKE_CONTROL_POINT = new Pose(55, 85);
        public static Pose END_INTAKE_START_SCORE = new Pose(18, 82);

        // ROW 2
        public static Pose END_INTAKE_START_SCORE2 = new Pose(13, 59.5);
        public static Pose INTAKE_CONTROL_POINT2 = new Pose(62, 59);
        public static Pose INTAKE_CONTROL_SCORE_R2 = new Pose(28, 55);

        public static Pose INTAKE_GATE = new Pose(30, 64);
        public static Pose END_GATE = new Pose (5, 63);
    }

    @Configurable
    public static class AudienceSidePoses {
        public static Pose A_SCORE_TARGET = new Pose(53, 15);

        public static Pose A_ROW3_END = new Pose(10, 35);
        public static Pose A_ROw3_CONTROL = new Pose(49, 35);
    }
}
