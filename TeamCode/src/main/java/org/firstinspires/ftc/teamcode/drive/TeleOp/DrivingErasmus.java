package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Driving")
public class DrivingErasmus extends LinearOpMode {

    DcMotorEx left = null;
    DcMotorEx right = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }
    private void initialization(){
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    boolean lastPressedStart = false;
    private void controlWheels(){

            float sp = gamepad1.right_bumper ? 0.4f : 1f;
            float ms = 0.8f;

            float leftPower = (-gamepad1.left_trigger + gamepad1.right_trigger - gamepad1.left_stick_y) * sp * ms;
            float rightPower = (gamepad1.left_trigger - gamepad1.right_trigger - gamepad1.left_stick_y) * sp * ms;
            left.setPower(leftPower);
            right.setPower(rightPower);
    }
    private void run(){
        controlWheels();
        debugTelemetry();
    }

    private void debugTelemetry(){
        telemetry.addData("roata stanga", left.getCurrentPosition());
        telemetry.addData("roata dreapta", right.getCurrentPosition());
        telemetry.addData("xleftstick", gamepad1.left_stick_x);
        telemetry.addData("yleftstick", gamepad1.left_stick_y);
        telemetry.update();
    }

}
