package org.firstinspires.ftc.teamcode.drive.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "Testing")
public class DualInverseMotorTest extends LinearOpMode
{
    DcMotorEx motor1, motor2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();
        waitForStart();

        while (opModeIsActive() && !isStopRequested())
            RunLoop();
    }

    private void Init()
    {
        motor1 = hardwareMap.get(DcMotorEx.class, "slot0");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor2 = hardwareMap.get(DcMotorEx.class, "slot1");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.setMsTransmissionInterval(250);
    }

    private void RunLoop()
    {
        motor1.setPower( gamepad1.right_stick_y);
        motor2.setPower(-gamepad1.right_stick_y);

        Telemetry();
    }

    private void Telemetry()
    {
        telemetry.addData("Motor 1 power:", motor1.getPower());
        telemetry.addData("Motor 2 power:", motor2.getPower());
    }
}