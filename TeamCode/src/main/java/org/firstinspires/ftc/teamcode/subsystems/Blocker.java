package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Blocker extends SubsystemBase{
    SimpleServo servoB;
    SimpleServo servoA;

    public double angle;
    
    public double position1;

    public double minDegree;
    public double maxDegree;

    Telemetry telemetry;
    public Blocker(HardwareMap hwMap, Telemetry telemetry) {
        servoB = new SimpleServo(hwMap, "servoB", minDegree, 180);
        servoA = new SimpleServo(hwMap, "servoA", minDegree, 180);
        servoB.setInverted(true);

    // NOTE: Angle will be calculated from the horizontal. Both minDegree and maxDegree will be > 0

    }
    // Calculates the position (from 0 to 1) based on the input angle relative to the max and min angles
    public double findPositionFromAngle(double inputAngle) {
        return ((inputAngle-minDegree) / (maxDegree - minDegree));
    }

    //Sends the blocker to the lowest permissible position
    public void positionZero() {
        servoB.setPosition(0);
        servoA.setPosition(0);
    }
    //Sends the blocker to the maximum permissible position (the block position)
    public void positionBlock() {
         servoB.setPosition(0.5);
         servoA.setPosition(0.5);
    }

    public void positionDefault() {
        servoB.setPosition(0.25);
        servoA.setPosition(0.25);
    }

    // Sends the blocker to a specific degree angle. Keep within the minDegree,maxDegree range.
    public void goToAngle(double inputAngle) {
        servoB.setPosition(findPositionFromAngle(inputAngle));
        servoA.setPosition(findPositionFromAngle(inputAngle));
    }

    public double get_angle() {
        angle = servoB.getAngle();
        return angle;
    }
    public double get_position() {
        position1 = servoB.getPosition();
        return position1;
    }

}