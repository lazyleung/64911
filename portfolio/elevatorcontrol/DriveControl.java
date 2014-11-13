/*
ECE649 FALL 2014
Group 11
Jonathan Leung/jkleung1
Eric Newhall/enewhall
Mengzhe Li/mzli 
Ting Xu/tingx
*/

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.*;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class DriveControl extends Controller implements TimeSensitive {

    // States
    public static enum State {
        STOP,
        FAST_DOWN,
        SLOW_DOWN,
        LEVEL_DOWN,
        FAST_UP,
        SLOW_UP,
        LEVEL_UP,
    }

    public static enum CommitPoint {
        REACHED,
        NOTREACHED,
        PASSED
    }

    // defines (in microseconds) how often this module should send out messages
    // and update its state
    SimTime period;

    // physical interface
    DriveSpeedPayload.ReadableDriveSpeedPayload localSpeed;
    DrivePayload.WriteableDrivePayload          localDrive;

    // output network
    DriveSpeedCanPayloadTranslator          mDriveSpeed;

    //input network
    CarLevelPositionCanPayloadTranslator    mCarLevelPosition;
    SafetySensorCanPayloadTranslator        mEmergencyBrake;
    DesiredFloorCanPayloadTranslator        mDesiredFloor;
    CarWeightCanPayloadTranslator           mCarWeight;

    LevelingCanPayloadTranslator            mLevelU;
    LevelingCanPayloadTranslator            mLevelD;
    HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitU;
    HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitD;

    DoorClosedCanPayloadTranslator          mDoorClosedFL;
    DoorClosedCanPayloadTranslator          mDoorClosedFR;
    DoorClosedCanPayloadTranslator          mDoorClosedBL;
    DoorClosedCanPayloadTranslator          mDoorClosedBR;

    Utility.AtFloorArray                    mAtFloor;

    // variables
    State       currentState;
    Direction   desiredDirection = Direction.STOP;
    int         currentFloor = 0;
    boolean     doorClosed = true;
    CommitPoint commitPoint = CommitPoint.NOTREACHED;

    public DriveControl(SimTime period, boolean verbose) {
        super("DriveControl", verbose);

        // Set local variables
        this.period = period;

        // Init physical interfaces
        localSpeed = DriveSpeedPayload.getReadablePayload();
        physicalInterface.registerTimeTriggered(localSpeed);
        physicalInterface.sendTimeTriggered(localDrive, period);

        //instantiate message objects
        ReadableCanMailbox r;
        WriteableCanMailbox w;

        // Output messages
        w = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(w);
        mDriveSpeed.setSpeed(localSpeed.speed());
        mDriveSpeed.setDirection(localSpeed.direction());
        canInterface.sendTimeTriggered(w, period);

        // Single messages
        r = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(r);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
        mEmergencyBrake = new SafetySensorCanPayloadTranslator(r);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(r);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(r);
        canInterface.registerTimeTriggered(r);

        // Up Down Message pairs
        r = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
        mLevelU = new LevelingCanPayloadTranslator(r, Direction.UP);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
        mLevelD = new LevelingCanPayloadTranslator(r, Direction.DOWN);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
        mHoistwayLimitU = new HoistwayLimitSensorCanPayloadTranslator(r, Direction.UP);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
        mHoistwayLimitD = new HoistwayLimitSensorCanPayloadTranslator(r, Direction.DOWN);
        canInterface.registerTimeTriggered(r);

        // Front Back Left Right Message pairs
        r = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
        mDoorClosedFL = new DoorClosedCanPayloadTranslator(r, Hallway.FRONT, Side.LEFT);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
        mDoorClosedFR = new DoorClosedCanPayloadTranslator(r, Hallway.FRONT, Side.RIGHT);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
        mDoorClosedBL = new DoorClosedCanPayloadTranslator(r, Hallway.BACK, Side.LEFT);
        canInterface.registerTimeTriggered(r);

        r = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
        mDoorClosedBR = new DoorClosedCanPayloadTranslator(r, Hallway.BACK, Side.RIGHT);
        canInterface.registerTimeTriggered(r);

        mAtFloor = new Utility.AtFloorArray(canInterface);
        // Every floor & hallway Message
    
        // initialize the controller to the STOP state
        currentState = State.STOP;
        doStop();

        // start a timer to interrupt based on the period
        timer.start(period);
    }

    public void timerExpired(Object callbackData) {
        // Update local variables
        if(mAtFloor.getCurrentFloor() != -1)
            currentFloor = mAtFloor.getCurrentFloor();

        int temp = currentFloor - mDesiredFloor.getFloor();
        if(temp < 0) {
            desiredDirection = Direction.UP;
        } else if(temp > 0) {
            desiredDirection = Direction.DOWN;
        } else if(temp == 0){
            desiredDirection = Direction.STOP;
        }

        doorClosed = mDoorClosedFL.getValue() && mDoorClosedFR.getValue() && mDoorClosedBL.getValue() && mDoorClosedBR.getValue();

        updateCommitPoint();

        log("currentFloor " + currentFloor);
        log("desiredDirection " + desiredDirection);
        log("commitPoint " + commitPoint);

        log("Executing state " + currentState);
        State nextState = currentState;
        switch (currentState) {
            case STOP:
                doStop();
                if(doorClosed == true && mCarWeight.getValue() < Elevator.MaxCarCapacity && desiredDirection == Direction.UP && mEmergencyBrake.getValue() != true) { //#transition 'T6.1'
                    log("T6.1");
                    nextState = State.SLOW_UP;
                } else if(doorClosed == true && mCarWeight.getValue() < Elevator.MaxCarCapacity && desiredDirection == Direction.DOWN && mEmergencyBrake.getValue() != true) { //#transition 'T6.5'
                    log("T6.5");
                    nextState = State.SLOW_DOWN;
                } else if(mLevelU.getValue() == false && mEmergencyBrake.getValue() != true) { //#transition 'T6.9'
                    log("T6.9");
                    nextState = State.LEVEL_UP;
                } else if(mLevelD.getValue() == false && mEmergencyBrake.getValue() != true) { //#transition 'T6.10'
                    log("T6.10");
                    nextState = State.LEVEL_DOWN;
                }
                break;
            case FAST_DOWN:
                doFastDown();
                if(mEmergencyBrake.getValue() == true || commitPoint == CommitPoint.REACHED || desiredDirection == Direction.UP) { //#transition 'T6.14'
                    log("T6.14");
                    nextState = State.SLOW_DOWN;
                }
                break;
            case SLOW_DOWN:
                doSlowDown();
                if(mEmergencyBrake.getValue() == true || desiredDirection == Direction.UP) { //#transition 'T6.8'
                    log("T6.8");
                    nextState = State.STOP;
                } else if(localSpeed.speed() == DriveObject.SlowSpeed && commitPoint == CommitPoint.NOTREACHED) { //#transition 'T6.13'
                    log("T6.13");
                    nextState = State.FAST_DOWN;
                } else if(desiredDirection == Direction.STOP && DriveObject.SlowSpeed >= localSpeed.speed()) { //#transition 'T6.6'
                    log("T6.6");
                    nextState = State.LEVEL_DOWN;
                }
                break;
            case LEVEL_DOWN:
                doLevelDown();
                if(mEmergencyBrake.getValue() == true || (mLevelU.getValue() == true && mLevelD.getValue() == true && DriveObject.LevelingSpeed >= localSpeed.speed())) { //#transition 'T6.7'
                    log("T6.7");
                    nextState = State.STOP;
                }
                break;
            case FAST_UP:
                doFastUp();
                if(mEmergencyBrake.getValue() == true || commitPoint == CommitPoint.REACHED || desiredDirection == Direction.DOWN) { //#transition 'T6.12'
                    log("T6.12");
                    nextState = State.SLOW_UP;
                }
                break;
            case SLOW_UP:
                doSlowUp();
                if(mEmergencyBrake.getValue() == true || desiredDirection == Direction.DOWN) { //#transition 'T6.4'
                    log("T6.4");
                    nextState = State.STOP;
                } else if (localSpeed.speed() == DriveObject.SlowSpeed && commitPoint == CommitPoint.NOTREACHED) { //#transition 'T6.11'
                    log("T6.11");
                    nextState = State.FAST_UP;
                } else if(desiredDirection == Direction.STOP && DriveObject.SlowSpeed >= localSpeed.speed()) { //#transition 'T6.2'
                    log("T6.2");
                    nextState = State.LEVEL_UP;
                }
                break;
            case LEVEL_UP:
                doLevelUp();
                if(mEmergencyBrake.getValue() == true || (mLevelU.getValue() == true && mLevelD.getValue() == true && DriveObject.LevelingSpeed >= localSpeed.speed())) { //#transition 'T6.3'
                    log("T6.3");
                    nextState = State.STOP;
                }
                break;
            default:
                throw new RuntimeException("Unrecognized state " + currentState);
        }

        //alwasy set mDriveSpeed to localSpeed
        mDriveSpeed.setSpeed(localSpeed.speed());
        mDriveSpeed.setDirection(localSpeed.direction());
        log("DriveSpeed=" + localSpeed.speed());
        log("Direction=" + localSpeed.direction());

        //advance to the next state we have computed
        if (currentState != nextState) {
            log ("Transition from " + currentState + " --> " + nextState);
        }
        currentState = nextState;

        // we start the timer to interrupt us again at the next period
        timer.start(period);
    }

    // Actions for STOP state
    private void doStop() {
        localDrive.set(Speed.STOP, Direction.STOP);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Actions for FAST_DOWN state
    private void doFastDown() {
        localDrive.set(Speed.FAST, Direction.DOWN);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Actions for SLOW_DOWN state
    private void doSlowDown() {
        localDrive.set(Speed.SLOW, Direction.DOWN);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Actions for LEVEL_DOWN state
    private void doLevelDown() {
        localDrive.set(Speed.LEVEL, Direction.DOWN);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Actions for FAST_UP state
    private void doFastUp() {
        localDrive.set(Speed.FAST, Direction.UP);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Actions for SLOW_UP state
    private void doSlowUp() {
        localDrive.set(Speed.SLOW, Direction.UP);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Actions for LEVEL_UP state
    private void doLevelUp() {
        localDrive.set(Speed.LEVEL, Direction.UP);
        // mDrive is set in timerExpired
        // mDriveSpeed is set in timerExpired
    }

    // Formula: x = x0 - 0.5 * v0^2 / a
    private void updateCommitPoint() {
        double targetPos = (double)(mDesiredFloor.getFloor() - 1) * 500;
        double pos = (double)mCarLevelPosition.getPosition();
        double finalPos = 0;
        double v0 = localSpeed.speed();
        double a = DriveObject.Acceleration;

        if(localSpeed.direction() == Direction.UP) {
            finalPos = pos + 0.5 * v0 * v0 / a;
            if (finalPos <  targetPos - 10 && v0 > DriveObject.SlowSpeed) {
                commitPoint = CommitPoint.NOTREACHED;
            } else if (finalPos < targetPos && finalPos >= targetPos - 10){
                commitPoint = CommitPoint.REACHED;
            } else if (finalPos >= targetPos){
                commitPoint = CommitPoint.PASSED;
            }
        } else if (localSpeed.direction() == Direction.DOWN) {
            finalPos = pos - 0.5 * v0 * v0 / a;
            if (finalPos >  targetPos + 10 && v0 > DriveObject.SlowSpeed) {
                commitPoint = CommitPoint.NOTREACHED;
            } else if (finalPos > targetPos && finalPos <= targetPos + 10){
                commitPoint = CommitPoint.REACHED;
            } else if (finalPos <= targetPos){
                commitPoint = CommitPoint.PASSED;
            }
        }
    }

}