package org.firstinspires.ftc.teamcode.archive;
//library imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotControls.depositStateMachine;
import org.firstinspires.ftc.teamcode.hwMecanum;
import org.firstinspires.ftc.teamcode.robotControls.lift;
import org.firstinspires.ftc.teamcode.te.vision;


/*
This is the test AUTO. At all expense, please do not use this for the final robot. Time based
park, carousel, etc.
TODO: This auto does not use roadrunner or PID. This is a backup auto.
 */
public class auto1 extends LinearOpMode {
    public hwMecanum robot;
    public lift robotlift;
    public vision vision1;
    public static final double ticksPerInch=537.7/11.87373601358268;
    public depositStateMachine deposit1;

    public void runOpMode() throws InterruptedException{

        initialize();

        waitForStart();


        while (opModeIsActive()){




        }

    }

    public void initialize(){
        robot = new hwMecanum(hardwareMap);
        robot.init(hardwareMap);
        deposit1=new depositStateMachine();
        deposit1.initDeposit(hardwareMap);
        vision1=new vision();
        vision1.initVision(hardwareMap);
    }


}
