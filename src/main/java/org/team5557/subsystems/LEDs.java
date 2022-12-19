package org.team5557.subsystems;

import org.team5557.Constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private final CANdle mCandle = new CANdle(Constants.ports.candle);
    private double timestamp = 0.0;

    private final boolean mUseSmartdash = false; // if we want to manual control lights using shuffleboard

    private LEDStatus mUnderglowStatus = new LEDStatus(Constants.ports.underglow_start_index, Constants.ports.underglow_end_index);

    // shuffleboard selectors
    
    private SendableChooser<State> mUnderglowChooser;

    // led states
    public enum State {
        OFF("OFF", -100, null),
        RAINBOW("RAINBOW", 0, new RainbowAnimation(1.0, 1.0, 0)),
        BLUE_ALLIANCE("BLUE_ALLIANCE", 0, new LarsonAnimation(0, 0, 255, 0, 1.0, 0, BounceMode.Center, 7)),
        RED_ALLIANCE("RED_ALLIANCE", 0, new LarsonAnimation(255, 0, 0, 0, 1.0, 0, BounceMode.Center, 7)),
        IDLE("BASIC_BLUE", -1, new ColorFlowAnimation(0, 0, 255, 0, 0.5, 0, Direction.Forward)),
        COPILOT_FREE_DRIVE("YELLOW_FLASH", 10, new StrobeAnimation(0,255,50, 0, 0.25, 0)),
        COPILOT_FOLLOWING("GREEN_FLASH", 10, new StrobeAnimation(0,255,50,0,0.5,0)),
        COPILOT_AT_TARGET("GREEN_STROBE", 60, new StrobeAnimation(0,255,50, 0, 0.9, 0));

        String name; // name of state
        Animation animation;
        int priority;
        int duration;

        private State(String name, int priority, Animation animation) {
            this.name = name;
            this.priority = priority;
            this.animation = animation;
        }

        public String getName() {
            return name;
        }
    }

    public LEDs() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.tunable_readout_key);
        configureCandle(); // set CTRE configurations for CANdle

        // create sendable choosers for shuffleboard
        if (mUseSmartdash) {
            mUnderglowChooser = new SendableChooser<>();
            for (State state : State.values()) {
                mUnderglowChooser.addOption(state.getName(), state);
            }
            mUnderglowChooser.setDefaultOption("OFF", State.OFF);
            tab.add("Underglow", mUnderglowChooser).withSize(2, 1).withPosition(2, 0);
        }
    }

    @Override
    public void periodic() {
        outputTelemtry();
        timestamp = Timer.getFPGATimestamp(); // update timestamp for color cycling
        if (mUseSmartdash) { // pull states from smartdash
            requestStateFromDashboard(mUnderglowChooser.getSelected());
        }
        updateState();
    }

    public void updateState() {
        updateUnderglowLeds();
    }

    private void updateUnderglowLeds() {
        if (timestamp >= mUnderglowStatus.statusExpiryTime) {
            mUnderglowStatus.flush();
        }
        mUnderglowStatus.state.animation.setLedOffset(mUnderglowStatus.colorIndex);
        mUnderglowStatus.state.animation.setLedOffset(mUnderglowStatus.LEDCount);
        mCandle.animate(mUnderglowStatus.state.animation);
    }

    // setter functions
    public void requestState(State underglowState) {
        mUnderglowStatus.addRequest(underglowState);
        //this is a function for all of the states
    }

    //this is bad code just ignore that its here and live with it
    /**
     * Use this method when requesting a state from the dashboard
     * @param fromDashboard - overides
     * @param underglowState
     */
    private void requestStateFromDashboard(State underglowState) {
        mUnderglowStatus.addDashboardRequest(underglowState);
    }

    /**
     * Adds a request to the underglow queu; it will be regulated by the priority of the state
     * @param state - requested LED state
     */
    public void requestUnderglowState(State state) {
        mUnderglowStatus.addRequest(state);
    }

    // apply configuration to candle
    private void configureCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, Constants.klong_CAN_TimeoutMs);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
    }

    public void clearAnimation() {
        mCandle.clearAnimation(0);
    }

    // getter functions
    public State getUnderglowState() {
        return mUnderglowStatus.state;
    }

    public boolean getUsingSmartdash() {
        return mUseSmartdash;
    }

    private void outputTelemtry() {
    }

    // class for holding information about each section
    private class LEDStatus {
        private State state = State.OFF; // current state
        private double statusExpiryTime = Timer.getFPGATimestamp(); // timestampe of last color cycle
        private int colorIndex = 0; // tracks current color in array
        private int startIDx, LEDCount; // start and end of section

        public LEDStatus(int startIndex, int endIndex) {
            startIDx = startIndex;
            LEDCount = endIndex - startIndex;
        }

        public void addRequest(State request) {
            if(request.priority > state.priority && request != state) {
                state = request;
            }
            statusExpiryTime = Timer.getFPGATimestamp() + 0.1;
        }

        public void addDashboardRequest(State request) {
            state = request;
        }

        public void flush() {
            state = State.IDLE;
        }
    }
}