package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class LeftSplineValues {
    public static AToHighOne a1ToHighOne;
    public static class AToHighOne {
        public static double x1 = 58.6,
                            y1 =-6.3,
                            heading1 =310.5;
        public static Vector2d aHighVector = new Vector2d(x1, y1);
    }




    public static BToConeOne b2ToConeTwo;
    public static class BToConeOne {
        public static double x2 = 54.1,
                            y2 =25.4,
                            heading2 =90;
        public static Vector2d aConeVector = new Vector2d(x2, y2);
    }
    public static BToMid b3ToMid;
    public static class BToMid {
        public static double x3 = 47.94,
                y3 =-4.85,
                heading3 =219;
        public static Vector2d bMidVector = new Vector2d(x3, y3);
    }




    public static CToConeOne c4ToCone;
    public static class CToConeOne {
        public static double x2 = 47.2,
                y2 =24.7,
                heading2 =90;
        public static Vector2d bConeVector = new Vector2d(x2, y2);
    }
public static CToMid c5ToMid;
    public static class CToMid {
        public static double x3 = 43.84,
                y3 =-5.79
                ,
                heading3 =219;
        public static Vector2d cMidVector = new Vector2d(x3, y3);
    }




    public static DToConeOne d6ToCone;
    public static class DToConeOne {
        public static double x2 = 40.,
                y2 =25,
                heading2 =90;
        public static Vector2d cConeVector = new Vector2d(x2, y2);
    }



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
