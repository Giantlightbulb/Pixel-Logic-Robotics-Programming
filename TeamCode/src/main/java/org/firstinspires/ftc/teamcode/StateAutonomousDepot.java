package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;

@Autonomous(name="StateAutonomousDepot", group="Autonomous")

public class StateAutonomousDepot extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        robot.init(hardwareMap);
        int targetPosition;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        //Approximate Speed at full power: 36 inches / 1.75 seconds
        // Left motion motor 2 positive, motor 4 negative
        // Forward motion motor 1 positive motor 3 negative
        //Gyro calibating
        robot.modernRoboticsI2cGyro.calibrate();

        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        //Initiate Servos
        robot.latch.setPosition(0.65); // latched
        robot.mascot.setPosition(0.8);//mascot up

        // Start Button
        waitForStart();
        robot.timer.reset();

        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //END INITIALIZATION---------------------------------------------------------------------


        //Left Motion (hopefully) - to clear the lander.
        DriveForwardDistance(robot.frontDrive, robot.backDrive,0.2, 600,10.0);

        sleep(3000);


        //Forward Motion (hopefully) - approach Sampling, part 1
        DriveForwardDistance(robot.leftDrive, robot.rightDrive,0.3, 4000,10.0);

        sleep(3000);

        //Left Motion (hopefully) - approach Sampling, part 2
        DriveForwardDistance(robot.frontDrive, robot.backDrive,0.3, 2000,10.0);

        sleep(3000);

        //Go Sample yoself

        //Left Motion (hopefully) - approach Sampling, part 2


        /*robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);
        targetPosition = 5000;
        while((robot.leftDrive.getCurrentPosition() < targetPosition) || (robot.rightDrive.getCurrentPosition() < targetPosition)){

        }
        */
        // end
    }
    public void DriveForwardDistance(DcMotor firstMotor, DcMotor secondMotor, double power, int distance, double timeOut){

        // Reset encoders
        firstMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Target Position b
        firstMotor.setTargetPosition(distance);
        secondMotor.setTargetPosition(distance);

        // Set to RUN_TO_POSITION mode
        firstMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        secondMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Drive Power
        firstMotor.setPower(power);
        secondMotor.setPower(power);
        robot.timer.reset();

        while((opModeIsActive())&&(firstMotor.isBusy() || secondMotor.isBusy())){
            // wait
        }

        firstMotor.setPower(0);
        secondMotor.setPower(0);

        firstMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
