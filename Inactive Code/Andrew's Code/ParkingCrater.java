package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ParkingCrater extends LinearOpMode {

    HardwareOmni robot = new HardwareOmni(); // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     SLOWER_SPEED = 0.2;

    public void runOpMode() {
    /*
    rotate to face crater
   Find location
*/
        // EASY OPTION
        robot.motor1.setPower(FORWARD_SPEED);
        robot.motor3.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <4.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update()
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);


        //COMPLEX OPTION
        double distanceLeft = 12 (or other random number)
        while (distanceLeft > 0) {
            if (robotSeen) {
                wait 5 seconds
                if robotSeen {
                    seek alternate route
                }
            } else {
                move robot x distance // (small x for increments)
                distanceLeft -= x
            }
        }
    */
    }
}