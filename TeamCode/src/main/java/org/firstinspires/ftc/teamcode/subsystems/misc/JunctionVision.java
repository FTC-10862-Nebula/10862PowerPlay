package org.firstinspires.ftc.teamcode.subsystems.misc;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Util;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.FFRectDetector;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.TeamMarkerPipeline;

import java.util.logging.Level;

public class JunctionVision extends SubsystemBase {
    private final Telemetry telemetry;
    private final FFRectDetector duckDetector;
    private TeamMarkerPipeline.Position currentPos;

    public JunctionVision(HardwareMap hw, Telemetry tl) {
        duckDetector = new FFRectDetector(hw, tl);
        duckDetector.init();

        duckDetector.setLeftRectangle(0.12, 0.25);
        duckDetector.setCenterRectangle(0.51, .25);
        duckDetector.setRightRectangle(0.90, .25);
        telemetry = tl;
        currentPos = duckDetector.getPosition();
    }


    @Override
    public void periodic() {
        currentPos = duckDetector.getPosition();
        Util.logger(this, telemetry, Level.INFO, "Duck Position: ", duckDetector.getPosition());
    }

    public TeamMarkerPipeline.Position getCurrentPosition() {
        return currentPos;
    }
}