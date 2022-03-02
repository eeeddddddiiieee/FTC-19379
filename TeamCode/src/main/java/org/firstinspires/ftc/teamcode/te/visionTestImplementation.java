package org.firstinspires.ftc.teamcode.te;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hwMecanum;

@Config
@Autonomous (name="visiontestimplementation")
public class visionTestImplementation extends LinearOpMode {

    public void runOpMode(){
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        vision vision1=new vision();
        vision1.initVision(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(vision1.webcam, 10);

        waitForStart();
        while (opModeIsActive())
        {
            vision1.checkTE();
            switch (vision1.getPosition()){
                case LEFT:
                    telemetry.addData("Position","LEFT");
                case CENTER:
                    telemetry.addData("Position","LEFT");

                case RIGHT:
                    telemetry.addData("Position","LEFT");

            }
            telemetry.addData("Area", vision1.getArea());
            telemetry.addData("Position",vision1.getCenter());

            telemetry.update();
        }
    }
}
