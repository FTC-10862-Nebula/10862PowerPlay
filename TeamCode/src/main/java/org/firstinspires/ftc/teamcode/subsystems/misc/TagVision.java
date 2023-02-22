package org.firstinspires.ftc.teamcode.subsystems.misc;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class TagVision extends SubsystemBase {
    OpenCvCamera camera;
    private Telemetry telemetry;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.4;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;
    int tagFoundNum = 0;

    public TagVision(HardwareMap hw, Telemetry tl)
    {
        this.telemetry=tl;

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                tl.addLine("ERROR for TagVision!");
            }
        });
    }

    @Override
    public void periodic()
    {
        updateTagOfInterest();
        tagToTelemetry();
    }

    public int getTag() {
//        return tagFoundNum;

        if (tagFoundNum == 1) {
            telemetry.addLine("Found 1 - Left");
            return 1;

        } else if (tagFoundNum == 2) {
            telemetry.addLine("Found 2 - Mid");
            return 2;
        } else if (tagFoundNum == 3) {
            telemetry.addLine("Found 3 - Right");
            return 3;
        } else {
            telemetry.addLine("Returning 1 - Left (Not Found)");
            return 1;
        }
    }

    public void updateTagOfInterest() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        tagFound = false;
        if (currentDetections.size() == 0) return;

        for(AprilTagDetection tag : currentDetections) {
            if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                tagFound = true;
                tagFoundNum = tag.id;
            }
        }
    }

    public void tagToTelemetry()
    {
        if (tagFoundNum == 0) {
            telemetry.addLine("Tag In Sight: Not Found :( ");
            return;
        }
        else {
            telemetry.addData("Tag In Sight: ", tagFoundNum);
        }
        telemetry.addLine();
    }


    public void stopTagVision() {
        camera.closeCameraDevice();
    }

}
