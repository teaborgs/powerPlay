package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Driving")
public class TudorTeleOp2 extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;
    DcMotorEx liftMotor1, liftMotor2, plateMotor;
    Servo catcher;
    double suppress1;
    int cp1 = 0, pp = 0, cp2 = 0;
    double suppressRotate;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization() {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        catcher.setPosition(0);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void controlWheels() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * suppress1,
                -gamepad1.left_stick_x * suppress1
        ).rotated(-poseEstimate.getHeading());
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -(gamepad1.right_trigger - gamepad1.left_trigger) * suppressRotate
                )
        );
    }

    boolean standing = true;

    private void resetArmLocalization() {
        if(gamepad2.right_stick_button) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void resetPlateLocalization() {
        if(gamepad2.start) {
            plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }
    private void controlArm() {
        if (gamepad2.left_stick_y != 0) {
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp1 = liftMotor1.getCurrentPosition();
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp2 = liftMotor2.getCurrentPosition();
            if(liftMotor1.getTargetPosition() >= 0f && gamepad2.left_stick_y > 0) liftMotor1.setPower(0f);
            else liftMotor1.setPower(1f * gamepad2.left_stick_y);
            if(liftMotor2.getTargetPosition() >= 0f && gamepad2.left_stick_y > 0) liftMotor2.setPower(0f);
            else liftMotor2.setPower(1f * gamepad2.left_stick_y);
        } else   {
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(liftMotor1.isBusy()) {
                if (liftMotor1.getCurrentPosition() > -10 && liftMotor1.getTargetPosition() > -10)
                    liftMotor1.setPower(0f);
                else liftMotor1.setPower(1f);
            } else {
                if (liftMotor1.getCurrentPosition() > -10 && liftMotor1.getTargetPosition() > -10)
                    liftMotor1.setPower(0f);
                else liftMotor1.setPower(0.1f);
            }
            if(liftMotor2.isBusy()) {
                if (liftMotor2.getCurrentPosition() > -10 && liftMotor2.getTargetPosition() > -10)
                    liftMotor2.setPower(0f);
                else liftMotor2.setPower(1f);
            } else {
                if (liftMotor2.getCurrentPosition() > -10 && liftMotor2.getTargetPosition() > -10)
                    liftMotor2.setPower(0f);
                else liftMotor2.setPower(0.1f);
            }
        }
        if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pp = plateMotor.getCurrentPosition();
            plateMotor.setPower(1f * (gamepad2.right_trigger - gamepad2.left_trigger));
        } else {
            plateMotor.setTargetPosition(pp);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(plateMotor.isBusy())
                plateMotor.setPower(1f);
            else
                plateMotor.setPower(0f);
        }
    }

    boolean lastPressedCatch = false;

    private void controlCatcher() {
        boolean left_bumper1_pressed = gamepad1.left_bumper;
        if (left_bumper1_pressed && !lastPressedCatch) {
            if (catcher.getPosition() == 0)
                catcher.setPosition(.4f);
            else
                catcher.setPosition(0);
        }
        lastPressedCatch = left_bumper1_pressed;
    }

    private void setPlateLevel() {
        if (gamepad2.right_bumper) {
            pp = -1000;
            plateMotor.setTargetPosition(-1000);
        } else if (gamepad2.left_bumper) {
            pp = 1000;
            plateMotor.setTargetPosition(1000);
        } else if (gamepad2.dpad_up) {
            pp = 0;
            plateMotor.setTargetPosition(0);
        }
        if(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.dpad_up)
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void setLiftLevel() {
        if (gamepad2.a) {
            pp = 0;
            plateMotor.setTargetPosition(0);
            cp1 = 0;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = 0;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = 0;
            plateMotor.setTargetPosition(0);
        } else if (gamepad2.x) {
            cp1 = -1272;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -1282;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = -1000;
            plateMotor.setTargetPosition(-1000);
        } else if (gamepad2.b) {
            cp1 = -770;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -770;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = -1000;
            plateMotor.setTargetPosition(-1000);
        } else if (gamepad2.y) {
            cp1 = -1757;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -1764;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = -1000;
            plateMotor.setTargetPosition(-1000);
        } else if (gamepad2.dpad_up) {
            cp1 = -1757;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -1764;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = 1000;
            plateMotor.setTargetPosition(1000);
        } else if(gamepad2.dpad_left) {
            cp1 = -1272;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -1282;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = 1000;
            plateMotor.setTargetPosition(1000);
        }
        else if(gamepad2.dpad_right) {
            cp1 = -770;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -770;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = 1000;
            plateMotor.setTargetPosition(1000);
        }
        else if(gamepad2.dpad_down) {
            pp = 0;
            plateMotor.setTargetPosition(0);
            cp1 = 0;
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = 0;
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
            pp = 0;
            plateMotor.setTargetPosition(0);
        }
        if (gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y || gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    private void suppressWheels() {
        if (gamepad1.right_bumper) {
            suppress1 = 0.5f;
            suppressRotate = 0.5f;
        } else {
            suppress1 = 1f;
            suppressRotate = 1f;
        }
    }

    private void autonomousArm() {
        if(gamepad2.back) {
            pp = 1423;
            plateMotor.setTargetPosition(1423);
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    private void run() {
        resetArmLocalization();
        resetPlateLocalization();
        suppressWheels();
        setPlateLevel();
        //setArmPos();
        controlWheels();
        setLiftLevel();
        controlArm();
        controlCatcher();
        debugTelemetry();
        autonomousArm();
    }

    private void debugTelemetry() {
        telemetry.addData("lift1", liftMotor1.getCurrentPosition());
        telemetry.addData("lift2", liftMotor2.getCurrentPosition());
        telemetry.addData("plate", plateMotor.getCurrentPosition());
        telemetry.addData("claw", catcher.getPosition());
        telemetry.addData("motorPower1", liftMotor1.getPower());
        telemetry.addData("motorBusy1", liftMotor1.isBusy());
        telemetry.addData("motorPower2", liftMotor2.getPower());
        telemetry.addData("motorBusy2", liftMotor2.isBusy());
        telemetry.addData("plateBusy1", plateMotor.isBusy());
        telemetry.addData("platePower", plateMotor.getPower());
        telemetry.update();
    }

}
