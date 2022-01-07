package org.firstinspires.ftc.teamcode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.TEDetector;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="TEtestauto", group="Auto")
public class TEtestauto extends LinearOpMode {

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    TEDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TEDetector detector = new TEDetector();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        robot.camera.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        robot.camera.setPipeline(detector);
        // Remember to change the camera rotation
        robot.camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);

        FtcDashboard.getInstance().startCameraStream(robot.camera, 20);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Analysis", detector.getLocation());
            telemetry.addData("region1",detector.region1value());
            telemetry.addData("region2",detector.region2value());

            telemetry.update();
/*
            switch (detector.getLocation()) {
                case LEFT:
                    //robot.arm2.setPosition(1.02 - hwMecanum.low);
                    //robot.arm1.setPosition(hwMecanum.low);
                    telemetry.addData("Position:", "Left");
                    telemetry.update();
                    break;
                case RIGHT:
                    //robot.arm2.setPosition(1.02 - hwMecanum.mid);
                    //robot.arm1.setPosition(hwMecanum.mid);
                    telemetry.addData("Position:", "Center");
                    telemetry.update();
                    break;
                case NONE:
                    //robot.arm2.setPosition(1.02 - hwMecanum.high);
                    //robot.arm1.setPosition(hwMecanum.high);
                    telemetry.addData("Position:", "Right");
                    telemetry.update();
                    break;
            }
*/
                robot.camera.stopStreaming();
            }
        }


    }
