package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomRed{
    Autonom auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone12,catchCone2,catchCone3,catchCone4,catchCone5,deliverCone345;

    public AutonomRed(Autonom auto){
        this.auto = auto;
        initializeTrajectories();
    }

    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .strafeRight(59)
                .strafeLeft(5)
                .back(8)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverPreload.end())
                .strafeRight(1.6)
                .forward(25.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .build();
        deliverCone12 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone1.end())
                .back(24)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.25, () -> AutoUtil.platePosition(auto.plateMotor,0))
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone12.end())
                .strafeRight(0.8)
                .forward(24)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.25, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
                .build();
        deliverCone345 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone1.end())
                .back(25)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.25, () -> AutoUtil.platePosition(auto.plateMotor,0))
                .build();
        catchCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone345.end())
                .strafeRight(0.5)
                .forward(26)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.25, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
                .build();
        catchCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone345.end())
                .forward(26)
                .addTemporalMarker( 0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.25, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
                .build();
        catchCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone345.end())
                .forward(26.7)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.25, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
    }

    public void runAuto(){
        auto.AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(200);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone12);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone12);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone3);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone345);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone4);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone345);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone5);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone345);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(30000);
    }
}
