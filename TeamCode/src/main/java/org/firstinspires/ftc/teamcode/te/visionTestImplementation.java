package org.firstinspires.ftc.teamcode.te;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hwMecanum;
import org.firstinspires.ftc.teamcode.lift;
import org.firstinspires.ftc.teamcode.te.vision;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.te.vision.barcodePosition;

public class visionTestImplementation extends LinearOpMode {
    public hwMecanum robot;
    public vision vision1;



    public void runOpMode(){
        hwMecanum robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);

        vision1=new vision();
        vision1.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(robot.camera, 10);
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
