package org.firstinspires.ftc.teamcode.PIDtuner;

import com.acmerobotics.dashboard.config.Config;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class TuningController {
    public static double MOTOR_TICKS_PER_REV = 537.6;
    public static double MOTOR_MAX_RPM = 340;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static double maxspeed = 306;

    public static double TESTING_MAX_SPEED = 0.9 * MOTOR_MAX_RPM;
    public static double TESTING_MIN_SPEED = 0.3 * MOTOR_MAX_RPM;


    // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
    // alphabetical order. Thus, we preserve the actual order of the process
    // Then we append Z just because we want it to show below the MOTOR_ and TESTING_ because
    // these settings aren't as important
    public static double ZSTATE1_RAMPING_UP_DURATION = 3.5;
    public static double ZSTATE2_COASTING_1_DURATION = 3.5;
    public static double ZSTATE3_RAMPING_DOWN_DURATION = 2;
    public static double ZSTATE4_COASTING_2_DURATION = 1;
    public static double ZSTATE5_RANDOM_1_DURATION = 2;
    public static double ZSTATE6_RANDOM_2_DURATION = 2;
    public static double ZSTATE7_RANDOM_3_DURATION = 2;
    public static double ZSTATE8_REST_DURATION = 1;

    enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    private StateMachine stateMachine;

    private ElapsedTime externalTimer = new ElapsedTime();

    private double currentTargetVelo = 0.0;

    public TuningController() {
        stateMachine = new StateMachineBuilder<State>()



                .state(State.COASTING_1)
                .transitionTimed(ZSTATE2_COASTING_1_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(maxspeed))

                .state(State.COASTING_2)
                .transitionTimed(ZSTATE4_COASTING_2_DURATION)
                .onEnter(() -> currentTargetVelo = rpmToTicksPerSecond(0))

                .exit(State.COASTING_1)

                .build();

        stateMachine.setLooping(true);
    }

    public void start() {
        externalTimer.reset();
        stateMachine.start();
    }

    public double update() {
        stateMachine.update();

        return currentTargetVelo;
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}
