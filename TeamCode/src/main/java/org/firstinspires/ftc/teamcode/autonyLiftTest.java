package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by agb on 1/9/2018.
 */

//@Autonomous(name = "lift test")

public class autonyLiftTest extends LinearOpMode
{
    DcMotor LiftMotor;

    Servo LeftGripperServo;
    Servo RightGripperServo;
    Servo JewelArmServo;

    ServoController ServoController1;

    @Override
    public void runOpMode()
    {
        double leftGripperHoldingOffsetFromHalf = 0.030; // smaller number is closed tighter
        double rightGripperHoldingOffsetFromHalf = 0.001; // larger number is closed tighter
        double jewelArmServoRestPosition = 0.232;
        double jewelArmServoDownPosition = 0.85;

        LiftMotor = hardwareMap.dcMotor.get("lift motor");

        LeftGripperServo = hardwareMap.servo.get("left gripper");
        RightGripperServo = hardwareMap.servo.get("right gripper");
        JewelArmServo = hardwareMap.servo.get("jewel arm");
        ServoController1 = hardwareMap.servoController.get("servo controller 1");

        ServoController1.pwmDisable();

        waitForStart();

        LiftMotor.setTargetPosition(7000);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(1);

        while(opModeIsActive() && LiftMotor.isBusy()) {}

        sleep(1000);
        telemetry.addData("RunMode: ", LiftMotor.getMode());
        telemetry.update();

        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(-1);

        while(opModeIsActive() && LiftMotor.isBusy()) {}
    }
}
