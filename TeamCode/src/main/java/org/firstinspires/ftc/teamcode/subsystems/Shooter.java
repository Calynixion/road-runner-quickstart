package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Configurable
public class Shooter extends SubsystemBase {

    MotorEx ShooterA;
    MotorEx ShooterB;
    Telemetry telemetry;
    MotorGroup shooter;
    public static double power = 0.55;
    public Shooter(HardwareMap hwMap, Telemetry telemetry){
        this.ShooterA = new MotorEx(hwMap,"ShooterA");
        this.ShooterB = new MotorEx(hwMap, "ShooterB");
        this.telemetry = telemetry;
        ShooterB.setInverted(true);
        shooter = new MotorGroup(ShooterA,ShooterB);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(double power_strength){
        shooter.set(power*power_strength);
    }
    public void stop(){
        shooter.set(0);
    }



}
