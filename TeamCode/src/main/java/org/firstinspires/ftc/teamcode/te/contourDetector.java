package org.firstinspires.ftc.teamcode.te;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class contourDetector extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private ffDetectorPipeline pipeline;
    private OpenCvCamera camera;

    private final int cameraMonitorViewId = this
            .hardwareMap
            .appContext
            .getResources().getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    hardwareMap.appContext.getPackageName()
            );

    @Override
    public void runOpMode() throws InterruptedException {
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new ffDetectorPipeline(telemetry, DEBUG));

        ffDetectorPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        ffDetectorPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int i)
            {
                //we do not care
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            String pos = "[POSITION]" + " " + pipeline.getBarcode();
            telemetry.addData("[TEAM ELEMENT] >>", pos);
            telemetry.update();
        }
    }
}
