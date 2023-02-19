package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class LeftSplineValues {
    public static AToHighOne a1ToHighOne;
    public static class AToHighOne {
        public static double x1 = 63.5,
                y1 =-1,
                heading1 =313.6;
//        public static Vector2d aHighVector = new Vector2d(x1, y1);
    }




    public static BToConeOne b2ToConeTwo;
    public static class BToConeOne {
        public static double x = 56.9,
                y2 =35.2,
                heading2 =90;
//        public static Vector2d aConeVector = new Vector2d(x, y2);
    }
    public static BToMid b3ToMidTwo;
    public static class BToMid {
        public static double x3 = 51.4,
                y3 = -0.8,
                heading3 =219;
//        public static Vector2d bMidVector = new Vector2d(x3, y3);
    }




    public static CToConeOne c4ToCone;
    public static class CToConeOne {
        public static double x2 = 56.455,
                y2 =37.355,
                heading2 =90;
//        public static Vector2d bConeVector = new Vector2d(x2, y2);
    }
    public static CToMid c5ToMid;
    public static class CToMid {
        public static double x3 = 51.6,
                y3 = 0.95,
                heading3 =219;
//        public static Vector2d cMidVector = new Vector2d(x3, y3);
    }




    public static DToConeOne d6ToCone;
    public static class DToConeOne {
        public static double x2 = 55,
                y2 =37.5,
                heading2 =90;
        public static Vector2d cConeVector = new Vector2d(x2, y2);
    }
//    public static DToMid d6ToMid;
//    public static class DToMid {
//        public static double x3 = 48.1,
//                y3 = 5.3,
//                heading3 =219;
//        public static Vector2d cMidVector = new Vector2d(x3, y3);
//    }
//
//
//
//    public static EToConeOne e7ToCone;
//    public static class EToConeOne {
//        public static double x2 = 56.6,
//                y2 =43,
//                heading2 =90;
//        public static Vector2d cConeVector = new Vector2d(x2, y2);
//    }


//    public static DToMid d7ToMid;
//    public static class DToMid {
//        public static double x3 = 47,
//                y3 =-5.7,
//                heading3 =219;
//        public static Vector2d dMidVector = new Vector2d(x3, y3);
//    }
//
//    public static DToConeOne d8ToCone;
//    public static class DToConeOne {
//        public static double x2 = 46.9,
//                y2 =25,
//                heading2 =90;
//        public static Vector2d dConeVector = new Vector2d(x2, y2);
//    }

}