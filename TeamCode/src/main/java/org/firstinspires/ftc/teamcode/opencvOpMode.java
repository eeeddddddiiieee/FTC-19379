package org.firstinspires.ftc.teamcode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Skystone Detecotor", group="Auto")
public class opencvOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        robot.camera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        opencvtest detector = new opencvtest(telemetry);
        robot.camera.setPipeline(detector);
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public  void onOpened(){
                robot.camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            public void onError(int errorCode) {
                //deez nuts!
            }

        });


        waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                robot.arm2.setPosition(1.02-hwMecanum.low);
                robot.arm1.setPosition(hwMecanum.low);
                telemetry.addData("Position:","Left");
                telemetry.update();
                break;
            case RIGHT:
                robot.arm2.setPosition(1.02-hwMecanum.mid);
                robot.arm1.setPosition(hwMecanum.mid);
                telemetry.addData("Position:","Center");
                telemetry.update();
                break;
            case NOT_FOUND:
                robot.arm2.setPosition(1.02-hwMecanum.high);
                robot.arm1.setPosition(hwMecanum.high);
                telemetry.addData("Position:","Right");
                telemetry.update();
                break;
        }
        robot.camera.stopStreaming();
    }
}


