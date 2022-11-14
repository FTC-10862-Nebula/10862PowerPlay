package org.firstinspires.ftc.teamcode.subsystems.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.subsystems.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.autons.Autons.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
public class VisionSubsystemNOUSE extends HardwareSubsystem {
    public AprilTagDetection tagOfInterest = null;

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private static final double fx = 578.272;
    private static final double fy = 578.272;
    private static final double cx = 402.145;
    private static final double cy = 221.506;

    // UNITS ARE METERS
    private static final double tagSize = 0.166;

    // Tag ID 1.2.3 from the 36h11 family
    private static final int LEFT = 1;
    private static final int MIDDLE = 2;
    private static final int RIGHT = 3;



    private boolean tagFound = false;

    public static class CameraConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();

        public static class Hardware {
            public boolean REVERSED = false;

        }
        public static class Value {
            public double
                    BLUE_WAREHOUSE             = 1, // Degrees
                    BLUE_CAROUSEL              = 1, // Degrees
                    RED_WAREHOUSE              = 1, // Degrees
                    RED_CAROUSEL               = 1; // Degrees
            public double CAMERA_WAIT_TIME_DOUBLE = 5;
            public long CAMERA_WAIT_TIME = (long) (CAMERA_WAIT_TIME_DOUBLE * 1000);
        }
    }

    public VisionSubsystemNOUSE(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        // Initialize OpenCvWebcam with live preview
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV ran into an error", errorCode);
            }
        });
    }

    @Override
    public void periodic() {
        updateTagOfInterest();
        printTagData();
        telemetry.update();
    }

    public void updateTagOfInterest() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        tagFound = false;
        if (currentDetections.size() == 0) return;

        for(AprilTagDetection tag : currentDetections) {
            if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                tagFound = true;
                tagOfInterest = tag;
            }
        }
    }

    public AprilTagDetection getTagOfInterest() {
        return tagOfInterest;
    }



    public void printTagData() {
        if (tagOfInterest == null) {
            telemetry.addLine("Tag not found");
            return;
        }

        if (tagFound) telemetry.addLine("Tag in sight");
        else telemetry.addLine("Tag seen before but not in sight");

        telemetry.addLine();

        telemetry.addData("Detected tag ID", tagOfInterest.id);
        telemetry.addData("Translation X in meters", tagOfInterest.pose.x);
        telemetry.addData("Translation Y in meters", tagOfInterest.pose.y);
        telemetry.addData("Translation Z in meters", tagOfInterest.pose.z);
        telemetry.addData("Rotation Yaw in degrees", Math.toDegrees(tagOfInterest.pose.yaw));
        telemetry.addData("Rotation Pitch degrees", Math.toDegrees(tagOfInterest.pose.pitch));
        telemetry.addData("Rotation Roll degrees", Math.toDegrees(tagOfInterest.pose.roll));
    }


}