package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.sql.Types.NULL;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision extends SubsystemBase {
    OpenCvCamera camera;
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
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;
    int tagFoundNum = 0;

    public Vision (HardwareMap hardwareMap, String webCamName)//, Telemetry tl)
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webCamName), cameraMonitorViewId);
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
        if (tagFound && tagFoundNum == 1) {
            return 1;
        } else if (tagFound && tagFoundNum == 2) {
            return 2;
        } else if (tagFound && tagFoundNum == 3) {
            return 3;
        } else {
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
//                tagOfInterest = tag;
            }
        }
    }

    public void tagToTelemetry()
    {
//        if (tagOfInterest == null) {
//            telemetry.addLine("Tag not found");
//            return;
//        }
//
//        if (tagOfInterest != null)
//        {
//            telemetry.addData("Tag In Sight: ", tagOfInterest.id);
//        }
//        else telemetry.addLine("Tag seen before but not in sight");
//
//        telemetry.addLine();
//
//        telemetry.addData("Detected tag ID", tagOfInterest.id);
//        telemetry.addData("Translation X in meters", tagOfInterest.pose.x);
//        telemetry.addData("Translation Y in meters", tagOfInterest.pose.y);
//        telemetry.addData("Translation Z in meters", tagOfInterest.pose.z);
//        telemetry.addData("Rotation Yaw in degrees", Math.toDegrees(tagOfInterest.pose.yaw));
//        telemetry.addData("Rotation Pitch degrees", Math.toDegrees(tagOfInterest.pose.pitch));
//        telemetry.addData("Rotation Roll degrees", Math.toDegrees(tagOfInterest.pose.roll));
    }
}
