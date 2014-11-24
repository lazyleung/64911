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
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;

public class HallButtonControl extends simulator.framework.Controller{

    //store the period for the controller
    private SimTime period;

    //additional internal state variables
    private SimTime counter = SimTime.ZERO;

    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;

    //send mHallCall messages
    private WriteableCanMailbox networkHallCallOut;
    private MyBooleanCanPayloadTranslator mHallCall;
    //private HallCallCanPayloadTranslator mHallCall;

    //received doorClosed messages
    private ReadableCanMailbox networkDoorClosedLeft;
    private ReadableCanMailbox networkDoorClosedRight;
    private DoorClosedCanPayloadTranslator mDoorClosedLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedRight;


    //received mAtFloor messages
    private ReadableCanMailbox networkAtFloor;
    //translator for the doorClosed message -- this translator is specific
    //to this messages, and is provided the elevatormodules package
    private AtFloorCanPayloadTranslator mAtFloor;

    //received mDesiredFloor messages
    private ReadableCanMailbox networkDesiredFloor;
    //translator for the doorClosed message -- this translator is specific
    //to this messages, and is provided the elevatormodules package
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    //local physical state
    private ReadableHallCallPayload localHallCall;
    private WriteableHallLightPayload localHallLight;

    //enumerate states
    private enum State {
        STATE_IDLE,
        STATE_REGISTER_CALL,
    }

    private State state = State.STATE_IDLE;

    private Direction desiredDirection;

    public HallButtonControl(SimTime period, int floor, Hallway hallway, Direction direction, boolean verbose) {
        super("HallButtonControl"+ReplicationComputer.makeReplicationString(floor, hallway, direction), verbose);
        this.period = period;
        this.hallway = hallway;
        this.direction = direction;
        this.floor = floor;

        // inputs
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);
        physicalInterface.registerTimeTriggered(localHallCall);

        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        mDoorClosedLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedLeft, hallway, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedLeft);
        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        mDoorClosedRight = new DoorClosedCanPayloadTranslator(networkDoorClosedRight, hallway, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedRight);

        networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID+ReplicationComputer.computeReplicationId(floor, hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
        canInterface.registerTimeTriggered(networkAtFloor);

        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);


        // outputs
        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);
        physicalInterface.sendTimeTriggered(localHallLight, period);

        networkHallCallOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallCall = new MyBooleanCanPayloadTranslator(networkHallCallOut);
        canInterface.sendTimeTriggered(networkHallCallOut, period);

        timer.start(period);
    }


    public HallButtonControl(int floor, Hallway hallway, Direction direction, SimTime period, boolean verbose) {
        super("HallButtonControl"+ReplicationComputer.makeReplicationString(floor, hallway, direction), verbose);
        this.period = period;
        this.hallway = hallway;
        this.direction = direction;
        this.floor = floor;

        // inputs
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);
        physicalInterface.registerTimeTriggered(localHallCall);

        networkDoorClosedLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.LEFT));
        mDoorClosedLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedLeft, hallway, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedLeft);
        networkDoorClosedRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, Side.RIGHT));
        mDoorClosedRight = new DoorClosedCanPayloadTranslator(networkDoorClosedRight, hallway, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedRight);

        networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID+ReplicationComputer.computeReplicationId(floor, hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor, hallway);
        canInterface.registerTimeTriggered(networkAtFloor);

        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);

        // outputs
        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);
        physicalInterface.sendTimeTriggered(localHallLight, period);

        networkHallCallOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
        mHallCall = new MyBooleanCanPayloadTranslator(networkHallCallOut);
        canInterface.sendTimeTriggered(networkHallCallOut, period);

        timer.start(period);
    }


    @Override
    public void timerExpired(Object callbackData) {
        State newState = state;

        if ((floor == 5) && (direction == Direction.DOWN)){
            //System.out.println("HALLCALL[5][DOWN] = "+mHallCall.getValue() + "     hallLight[5][DOWN] = "+localHallLight.lighted() + "    state = "+state);
            
        }
        
        
        if (mDesiredFloor.getFloor() > floor){
            desiredDirection = Direction.UP;
        } else if (mDesiredFloor.getFloor() < floor) {
            desiredDirection = Direction.DOWN;
        } else {
            desiredDirection = Direction.STOP;
        }

        switch (state) {
        case STATE_IDLE:
            // state actions for 'STATE_IDLE'
            localHallLight.set(false);
            mHallCall.set(false);

        //#transition 'T8.1'
            //if ((localHallCall.pressed() && mDoorClosedLeft.getValue() && mDoorClosedRight.getValue() ) ||
            //    (localHallCall.pressed() && ((!mDoorClosedLeft.getValue()) || (!mDoorClosedRight.getValue()))
            //         && mAtFloor.getValue() && (!desiredDirection.equals(direction))) ) {
            if (localHallCall.pressed()){
                newState = State.STATE_REGISTER_CALL;
            } else {
                newState = State.STATE_IDLE;
            }
            break;
        case STATE_REGISTER_CALL:
            // state actions for 'STATE_REGISTER_CALL'
            localHallLight.set(true);
            mHallCall.set(true);

        //#transition 'T8.2'
       // if (mAtFloor.getValue() && (desiredDirection.equals(direction)) && ((!mDoorClosedLeft.getValue()) || (!mDoorClosedRight.getValue()))) {
       if ((mDesiredFloor.getFloor() == floor) && mAtFloor.getValue() && ((mDesiredFloor.getHallway() == Hallway.BOTH) || (mDesiredFloor.getHallway() == hallway)) &&
           (mDesiredFloor.getDirection() == direction)){
                newState = State.STATE_IDLE;
        } else {
                newState = State.STATE_REGISTER_CALL;
            }
            break;
        default:
            throw new RuntimeException("State " + state + " was not recognized.");
        }

        //log the results of this iteration
        if (state == newState) {
            log("remains in state: ",state);
        } else {
            log("Transition: ",state,"->",newState);
        }

        //update the state variable
        state = newState;

        //report the current state
        setState(STATE_KEY, newState.toString());

        //schedule the next iteration of the controller
        //you must do this at the end of the timer callback in order to restart
        //the timer
        timer.start(period);
    }
}
