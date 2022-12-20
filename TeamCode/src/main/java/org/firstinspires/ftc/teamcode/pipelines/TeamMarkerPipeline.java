package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import java.util.logging.Level;
import java.util.logging.Logger;

@Config
public class TeamMarkerPipeline extends FFRectMarkerPipeline {

    public static double MIN = 105;
    public static double MAX = 110;

    public void setLeftRectangle(double x, double y) {
        setLeftRectHeightPercentage(y);
        setLeftRectWidthPercentage(x);
    }
    public void setCenterRectangle(double x, double y) {
        setCenterRectHeightPercentage(y);
        setCenterRectWidthPercentage(x);
    }
    public void setRightRectangle(double x, double y) {
        setRightRectHeightPercentage(y);
        setRightRectWidthPercentage(x);
    }
    public void setRectangleSize(int w, int h) {
        setRectangleHeight(h);
        setRectangleWidth(w);
    }

    public Position getPosition() {
        Util.logger(this, Level.INFO, "Left Avg: ", getLeftAverage());

        if(getLeftAverage() > getCenterAverage() && getLeftAverage() > getRightAverage()){
            return Position.LEFT;
        }
        else if(getCenterAverage() > getLeftAverage() && getCenterAverage() > getRightAverage()){
            return Position.MIDDLE;
        }
        else if(getRightAverage() > getLeftAverage() && getRightAverage() > getCenterAverage()){
            return Position.RIGHT;
        }
        else{
            return Position.RIGHT;
        }
    }

    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT,
    }

}