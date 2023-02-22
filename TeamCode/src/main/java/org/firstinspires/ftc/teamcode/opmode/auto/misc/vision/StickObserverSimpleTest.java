package org.firstinspires.ftc.teamcode.opmode.auto.misc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.misc.StickVision;
@Disabled
@Autonomous
public class StickObserverSimpleTest extends LinearOpMode {
    private Telemetry telemetry;

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        StickVision stickVision = new StickVision(hardwareMap, telemetry, this);
//      call the function to startStreaming
        stickVision.observeStick();

        waitForStart();
        while (opModeIsActive()) {

        }
//        stopStreaming
        stickVision.stopCamera();
    }
}

