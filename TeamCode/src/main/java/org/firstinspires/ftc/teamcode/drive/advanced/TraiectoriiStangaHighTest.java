package org.firstinspires.ftc.teamcode.drive.advanced;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

public class TraiectoriiStangaHighTest {
    AutonomStangaHighTest auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;
    public TraiectoriiStangaHighTest(AutonomStangaHighTest auto){
        this.auto = auto;
        initializeTrajectories();
    }
    Trajectory catchCone;

    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .strafeRight(54.25)
                .back(12.8)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        deliverCone = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        park3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(9.9)
                .turn(Math.toRadians(90))
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(1.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(13)
                .turn(Math.toRadians(90))
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(1.7, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(33.25)
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-2423))
                .turn(Math.toRadians(90))
                .addTemporalMarker(1.8, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(2.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                /// .addTemporalMarker(2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        catchCone = auto.mecanumDrive
                .trajectoryBuilder(deliverPreload.end())
                .forward(30)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.3, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .build();
    }
    boolean colorPass()
    {
        if(auto.AutoUtil.colorDet(auto.sensor) == true)
            return true;
        return false;
    }
    public void runAuto(int detected){
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(300);
        auto.AutoUtil.setClaw(auto.catcher, true);
        auto.mecanumDrive.followTrajectoryAsync(catchCone);
        if(colorPass())
         {
            // Cancel following
            auto.mecanumDrive.breakFollowing();

            // Stop the motors
            auto.mecanumDrive.setDrivePower(new Pose2d());
        }
    }
}
