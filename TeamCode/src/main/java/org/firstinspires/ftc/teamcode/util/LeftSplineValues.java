package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class LeftSplineValues {
    public static AToHighOne aToHighOne;
    public static class AToHighOne {
        public static double x1 = 58.6,
                            y1 =-6.2,
                            heading1 =310.1;
    }

    public static BToConeOne bToCone;
    public static class BToConeOne {
        public static double x2 = 54.6,
                            y2 =25.2,
                            heading2 =90;
    }

//    public static CToHighTwo cToHighTwo;
//    public static class CToHighTwo {
//        public static double x1 = 58.6,
//                y1 =-6.98,
//                heading1 =329.5;
//    }

    public static CToMid cToMid;
    public static class CToMid {
        public static double x3 = 47.7,
                y3 =-5.6,
                heading3 =219;
    }

    public static DToConeOne dToCone;
    public static class DToConeOne {
        public static double x2 = 46.9,
                y2 =25,
                heading2 =90;
    }

    public static EToMid eToMid;
    public static class EToMid {
        public static double x3 = 47.65,
                y3 =-5.64,
                heading3 =219;
    }

    public static FToConeOne fToCone;
    public static class FToConeOne {
        public static double x2 = 48,
                y2 =25,
                heading2 =90;
        public static Vector2d fVector = new Vector2d(x2, y2);
    }

    public static GToMid gToMid;
    public static class GToMid {
        public static double x3 = 47,
                y3 =-5.7,
                heading3 =219;
    }

    public static HToConeOne hToCone;
//    Vector2d fsd = new Vector2d(4,45);
    public static class HToConeOne {
        public static double x2 = 46.9,
                y2 =25,
                heading2 =90;
        public static Vector2d hVector = new Vector2d(x2, y2);
    }

}
