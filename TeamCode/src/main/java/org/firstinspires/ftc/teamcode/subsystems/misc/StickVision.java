package org.firstinspires.ftc.teamcode.subsystems.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.StickObserverPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class StickVision {
    private final OpenCvWebcam webcam;
//    private StickObserverPipeline stickPipeline = null;
    private StickObserverPipeline stickPipeline;

    private MatchOpMode matchOpMode;
    private LinearOpMode linearOpMode;

    private final Telemetry telemetry;
    public StickVision(HardwareMap hw, Telemetry tl, MatchOpMode opmode){
        //you can input  a hardwareMap instead of linearOpMode if you want
        this.telemetry = tl;
        this.matchOpMode = opmode;
        //initialize webcam

        int cameraMonitorViewId = hw
                .appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
    }
    public StickVision(HardwareMap hw, Telemetry tl, LinearOpMode opmode){
        //you can input  a hardwareMap instead of linearOpMode if you want
        this.telemetry = tl;
        this.linearOpMode = opmode;
        //initialize webcam

        int cameraMonitorViewId = hw
                .appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
    }
    public void observeStick(){
        //create the pipeline
        stickPipeline = new StickObserverPipeline();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.setPipeline(stickPipeline);
                //start streaming the camera
                webcam.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
                //if you are using dashboard, update dashboard camera view
                /*FtcDashboard.getInstance().startCameraStream(webcam, 5);*/

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("ERROR for New Junction!");
            }
        });
    }

    //stop streaming
    public void stopCamera(){
        webcam.stopStreaming();
    }

//    public Mat returnInput(Mat mat){
////        if (mat.cols())
//        return stickPipeline.processFrame(mat);
//    }

    public double getTurning(){
        return stickPipeline.getTurning();
    }
    public void periodic(){
        telemetry.addData("Turning Number:", stickPipeline.getTurning());
        telemetry.addData("Center of Pole:", stickPipeline.centerOfPole());

    }
}