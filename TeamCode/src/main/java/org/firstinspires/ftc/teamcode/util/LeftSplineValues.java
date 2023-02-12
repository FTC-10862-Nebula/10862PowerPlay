package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LeftSplineValues {
    public static AToHighOne aToHighOne;
    public static class AToHighOne {
        public static double x1 = 58.6,
                            y1 =-6.2,
                            heading1 =310.;
    }

    public static BToConeOne bToCone;
    public static class BToConeOne {
        public static double x2 = 54.08,
                            y2 =24.2,
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
        public static double x3 = 47.5,
                y3 =-6.5,
                heading3 =220.25;
    }

    public static DToConeOne dToCone;
    public static class DToConeOne {
        public static double x2 = 46.88,
                y2 =23.34,
                heading2 =90;
    }


}
