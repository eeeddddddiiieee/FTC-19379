package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class depositStateMachine {
    public depositState dstate1;
    public lift robotlift;
    public boolean intakeMode;
    public double intakePower;
    public static PIDCoefficients PID = new PIDCoefficients(0.008, 0, 0.0005);
    public PIDFController liftController1;
    public enum depositState{
        START,
        PRIME,
        MID,
        HIGH,
        DUMP,
        RETRACT
    }
    public void initDeposit(HardwareMap hardwareMap){
        dstate1= depositState.START;
        robotlift=new lift(hardwareMap);
        robotlift.init(hardwareMap);
        intakeMode=true;
    }

    public void runOpMode(){

    }
    public void updatePID(hwMecanum r1){
        robotlift.updateLift(r1.lift1.getCurrentPosition());
    }
    public void setLiftPosition(double pos){
        liftController1.setTargetPosition(Range.clip(pos, 0, 800));
    }


    public void deposit(hwMecanum robot, Gamepad gamepad1)throws InterruptedException{
        switch (dstate1) {
            case START:
                intakeMode=true;
                robot.bucket.setPosition(hwMecanum.bucketDown);
                robot.intakeServo.setPosition(hwMecanum.intakeDown);
                robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                if (robot.isCargo==true){
                    dstate1= depositState.PRIME;
                }
                break;
            case PRIME:
                robot.depositServo.setPosition(hwMecanum.depositClosed);
                robot.intakeServo.setPosition(hwMecanum.intakeUp);
                intakeMode=false;
                intakePower=.75;
                robot.bucket.setPosition(hwMecanum.bucketRaised);

                if (gamepad1.a){
                    dstate1= depositState.HIGH;
                }
                if (gamepad1.y){
                    dstate1= depositState.MID;
                }
                if (gamepad1.b){
                    dstate1= depositState.DUMP;
                }
                break;
            case MID:
                intakeMode=true;
                robotlift.setHeight(320,lift.resetMode.NO);
                //setLiftPosition(350);
                if (robotlift.getHeight()>250) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case HIGH:
                intakeMode=true;
                robotlift.setHeight(1000,lift.resetMode.NO);
                if (robotlift.getHeight()>750) {
                    dstate1 = depositState.DUMP;
                }
                break;
            case DUMP:
                intakePower=0;
                robot.bucket.setPosition(hwMecanum.bucketOut);
                if (gamepad1.x){
                    robot.depositServo.setPosition(hwMecanum.depositMidOpen);
                    sleep(400);
                    robot.bucket.setPosition(hwMecanum.bucketRaised);
                    if (robot.bucket.getPosition()>.4){
                    dstate1= depositState.RETRACT;
                    }
                }
                break;
            case RETRACT:
                intakeMode=true;
                robotlift.setHeight(0, lift.resetMode.YES);
                if (robotlift.getHeight()<50)
                {
                    robot.bucket.setPosition(hwMecanum.bucketDown);

                }
                dstate1=depositState.START;
                break;

            default: dstate1= depositState.START;
        }
        
        if (gamepad1.right_stick_button && dstate1 != depositState.RETRACT) {
            dstate1 = depositState.RETRACT;
        }

    }
    public depositState getDepositState(){
        return dstate1;
    }
}
