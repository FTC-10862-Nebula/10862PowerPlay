package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

public class SensorColor extends SubsystemBase implements HardwareDevice {

    private ColorSensor colorSensor;
    private Telemetry telemetry;

    public SensorColor(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public SensorColor(ColorSensor colorSensor, HardwareMap hardwareMap , Telemetry tl) {
        this.colorSensor = colorSensor;
        this.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        this.telemetry = tl;
    }

    public void periodic() {
//        telemetry.addData("\tAlpha:", alpha());
//        telemetry.addData("\tRed:", red());
//        telemetry.addData("\tGreen:", green());
//        telemetry.addData("\tBlue:", blue());



//        Util.logger(this, telemetry, Level.INFO, "\tColor Alpha:",colorSensor.alpha());
//        Util.logger(this, telemetry, Level.INFO, "\tColor Red:",colorSensor.red());
//        Util.logger(this, telemetry, Level.INFO, "\tColor Green:",colorSensor.green());
//        Util.logger(this, telemetry, Level.INFO, "\tColor Blue:",colorSensor.blue());
int[] color = getARGB();
        Util.logger(this, telemetry, Level.INFO, "\tColor Alpha:",color[0]);
        Util.logger(this, telemetry, Level.INFO, "\tColor Red:",color[1]);
        Util.logger(this, telemetry, Level.INFO, "\tColor Green:",color[2]);
        Util.logger(this, telemetry, Level.INFO, "\tColor Blue:",color[3]);


//        Util.logger(this, telemetry, Level.INFO, "\tColor Alpha:",alpha());
//        Util.logger(this, telemetry, Level.INFO, "\tColor Red:",red());
//        Util.logger(this, telemetry, Level.INFO, "\tColor Green:",green());
//        Util.logger(this, telemetry, Level.INFO, "\tColor Blue:",blue());
        telemetry.update();
    }

    public int[] HSVtoARGB(int alpha, float[] hsv) {
        int color = Color.HSVToColor(alpha, hsv);
        return new int[]{Color.alpha(color), Color.red(color), Color.green(color), Color.blue(color)};
    }

    public float[] RGBtoHSV(int red, int green, int blue, float[] hsv) {
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }

    public int[] getARGB() {
        return new int[]{alpha(), red(), green(), blue()};
    }

    public int alpha() {return colorSensor.alpha();}
    public int red() {return colorSensor.red();}
    public int green() {return colorSensor.green();}
    public int blue() {return colorSensor.blue();}

    public boolean grabbedBlueCone() {
        //TODO:Change the color
//        telemetry.addLine();
        return (red() > 150) && (green() > 150);
    }
    public boolean grabbedRedCone() {
        //TODO:Change the color
        return (red() > 150) && (green() > 150);
    }

    @Override
    public void disable() {colorSensor.close();}

    @Override
    public String getDeviceType() {return "Color Sensor";}

}