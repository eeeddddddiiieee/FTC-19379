package org.firstinspires.ftc.teamcode.te;

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
        while (opModeIsActive())
        {
            vision1.checkTE();
            switch (vision1.getPosition()){
                case LEFT:
                case CENTER:
                case RIGHT:
            }
        }
    }
}
