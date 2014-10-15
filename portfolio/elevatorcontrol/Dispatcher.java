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
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class Dispatcher extends simulator.framework.Controller{
	//store the period for the controller
    private SimTime period;
    
    //amount of time to stop as a floor.
    private final int dwellTime = 5;
    
	//received mAtFloor messages
    private ReadableCanMailbox[] networkAtFloors = new ReadableCanMailbox[10];
    private AtFloorCanPayloadTranslator[] mAtFloors = new AtFloorCanPayloadTranslator[10];
    private int[] AtFloorFloors = {1, 1, 2, 3, 4, 5, 6, 7, 7, 8};
    private Hallway[] AtFloorHallways = {Hallway.FRONT, Hallway.BACK, Hallway.BACK, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.BACK, Hallway.FRONT};
    
    //received mDoorClosed messages
    private ReadableCanMailbox networkDoorClosedFrontLeft;
    private ReadableCanMailbox networkDoorClosedFrontRight;
    private DoorClosedCanPayloadTranslator mDoorClosedFrontLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedFrontRight;
    private ReadableCanMailbox networkDoorClosedBackLeft;
    private ReadableCanMailbox networkDoorClosedBackRight;
    private DoorClosedCanPayloadTranslator mDoorClosedBackLeft;
    private DoorClosedCanPayloadTranslator mDoorClosedBackRight;
    
    //received mHallCall message
    private ReadableCanMailbox[] networkHallCalls = new ReadableCanMailbox[17];
    private HallCallCanPayloadTranslator[] mHallCalls =  new HallCallCanPayloadTranslator[17];
    private int[] HallCallFloors = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8};
    private Hallway[] HallCallHallways = {Hallway.FRONT,Hallway.FRONT,Hallway.BACK,Hallway.BACK,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.BACK,Hallway.BACK,Hallway.FRONT};
    private Direction[] HallCallDirections = {Direction.UP,Direction.UP,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.DOWN,};
    
    //receive mCarWeight message
    private ReadableCanMailbox networkCarWeight;
    @SuppressWarnings("unused")
	private CarWeightCanPayloadTranslator mCarWeight;
    
    //received mCarCall message
    private ReadableCanMailbox[] networkCarCalls = new ReadableCanMailbox[10];
    private CarCallCanPayloadTranslator[] mCarCalls = new CarCallCanPayloadTranslator[10];
    private int[] CarCallFloors = {1, 1, 2, 3, 4, 5, 6, 7, 7, 8};
    private Hallway[] CarCallHallways = {Hallway.FRONT, Hallway.BACK, Hallway.BACK, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.BACK, Hallway.FRONT};
    
    
    //send mDesiredFloor messages
    private WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private Hallway[] DesiredFloorHallways = {Hallway.BOTH, Hallway.BACK, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.BOTH, Hallway.FRONT};
    
    //send mDesiredDwell messages
    private WriteableCanMailbox networkDesiredDwellFront;
    private WriteableCanMailbox networkDesiredDwellBack;
    private DesiredDwellCanPayloadTranslator mDesiredDwellFront;
    private DesiredDwellCanPayloadTranslator mDesiredDwellBack;
	
    //enumerate states
    private enum State {
        STATE_INIT,
        STATE_GOTO_NEXT_FLOOR,
        STATE_EMERGENCY,
    }
	
    private State state = State.STATE_INIT;
    private int Target = 1;
    
	public Dispatcher(SimTime period, boolean verbose) {
		super("Dispatcher", verbose);
		this.period = period;
		//inputs
		networkDoorClosedFrontLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
        mDoorClosedFrontLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontLeft, Hallway.FRONT, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedFrontLeft);
        networkDoorClosedFrontRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
        mDoorClosedFrontRight = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontRight, Hallway.FRONT, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedFrontRight);
        networkDoorClosedBackLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
        mDoorClosedBackLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedBackLeft, Hallway.BACK, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedBackLeft);
        networkDoorClosedBackRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
        mDoorClosedBackRight = new DoorClosedCanPayloadTranslator(networkDoorClosedBackRight, Hallway.BACK, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedBackRight);
	
        for (int i = 0; i < mHallCalls.length; i++){
        	if (i < mAtFloors.length){
	        	networkAtFloors[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID+ReplicationComputer.computeReplicationId(AtFloorFloors[i], AtFloorHallways[i]));
	            mAtFloors[i] = new AtFloorCanPayloadTranslator(networkAtFloors[i], AtFloorFloors[i], AtFloorHallways[i]);
	            canInterface.registerTimeTriggered(networkAtFloors[i]);
        	}
        	if (i < mCarCalls.length){
	            networkCarCalls[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(CarCallFloors[i], CarCallHallways[i]));
	            mCarCalls[i] = new CarCallCanPayloadTranslator(networkCarCalls[i], CarCallFloors[i], CarCallHallways[i]);
	            canInterface.registerTimeTriggered(networkCarCalls[i]);
        	}
        	networkHallCalls[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(HallCallFloors[i], HallCallHallways[i], HallCallDirections[i]));
            mHallCalls[i] = new HallCallCanPayloadTranslator(networkHallCalls[i], HallCallFloors[i], HallCallHallways[i], HallCallDirections[i]);
            canInterface.registerTimeTriggered(networkHallCalls[i]);
        }
	
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);
        
        //outputs
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        networkDesiredDwellFront = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(Hallway.FRONT));
        mDesiredDwellFront = new DesiredDwellCanPayloadTranslator(networkDesiredDwellFront);
        canInterface.sendTimeTriggered(networkDesiredDwellFront, period);
        networkDesiredDwellBack = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(Hallway.BACK));
        mDesiredDwellBack = new DesiredDwellCanPayloadTranslator(networkDesiredDwellBack);
        canInterface.sendTimeTriggered(networkDesiredDwellBack, period);
        
        timer.start(period);
	}
	
	public Dispatcher(int numFloors, SimTime period, boolean verbose) {
		super("Dispatcher", verbose);
		this.period = period;
		//inputs
		networkDoorClosedFrontLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
        mDoorClosedFrontLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontLeft, Hallway.FRONT, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedFrontLeft);
        networkDoorClosedFrontRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
        mDoorClosedFrontRight = new DoorClosedCanPayloadTranslator(networkDoorClosedFrontRight, Hallway.FRONT, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedFrontRight);
        networkDoorClosedBackLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
        mDoorClosedBackLeft = new DoorClosedCanPayloadTranslator(networkDoorClosedBackLeft, Hallway.BACK, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedBackLeft);
        networkDoorClosedBackRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
        mDoorClosedBackRight = new DoorClosedCanPayloadTranslator(networkDoorClosedBackRight, Hallway.BACK, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedBackRight);
	
        for (int i = 0; i < mHallCalls.length; i++){
        	if (i < mAtFloors.length){
	        	networkAtFloors[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID+ReplicationComputer.computeReplicationId(AtFloorFloors[i], AtFloorHallways[i]));
	            mAtFloors[i] = new AtFloorCanPayloadTranslator(networkAtFloors[i], AtFloorFloors[i], AtFloorHallways[i]);
	            canInterface.registerTimeTriggered(networkAtFloors[i]);
        	}
        	if (i < mCarCalls.length){
	            networkCarCalls[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(CarCallFloors[i], CarCallHallways[i]));
	            mCarCalls[i] = new CarCallCanPayloadTranslator(networkCarCalls[i], CarCallFloors[i], CarCallHallways[i]);
	            canInterface.registerTimeTriggered(networkCarCalls[i]);
        	}
        	networkHallCalls[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(HallCallFloors[i], HallCallHallways[i], HallCallDirections[i]));
            mHallCalls[i] = new HallCallCanPayloadTranslator(networkHallCalls[i], HallCallFloors[i], HallCallHallways[i], HallCallDirections[i]);
            canInterface.registerTimeTriggered(networkHallCalls[i]);
        }
	
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);
        
        //outputs
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        networkDesiredDwellFront = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(Hallway.FRONT));
        mDesiredDwellFront = new DesiredDwellCanPayloadTranslator(networkDesiredDwellFront);
        canInterface.sendTimeTriggered(networkDesiredDwellFront, period);
        networkDesiredDwellBack = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(Hallway.BACK));
        mDesiredDwellBack = new DesiredDwellCanPayloadTranslator(networkDesiredDwellBack);
        canInterface.sendTimeTriggered(networkDesiredDwellBack, period);
        
        timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		
		int curFloor = -1;
		boolean AllDoorClosed = mDoorClosedFrontLeft.getValue() && mDoorClosedBackLeft.getValue() && mDoorClosedFrontRight.getValue() && mDoorClosedBackRight.getValue();
		boolean atFloor = false;
		boolean notAtFloor;
		for (int i = 0; i < mAtFloors.length; i++){
			atFloor = atFloor || mAtFloors[i].getValue();
			if (mAtFloors[i].getValue()){
				curFloor = AtFloorFloors[i];
			}
		}
		notAtFloor = !atFloor;
		
		log("curFloor="+curFloor+" atFloor="+atFloor+" AllDoorClosed="+AllDoorClosed);
		
		switch (state){
		case STATE_INIT:
			Target = 1;
			mDesiredFloor.set(Target, Direction.STOP, Hallway.BOTH);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			
			//#transition 'T11.1'
			if ((!AllDoorClosed) && atFloor){
				//log("------- Transition T11.1 -------------");
				newState = State.STATE_GOTO_NEXT_FLOOR;
			}
			//#transition 'T11.4'
			else if (notAtFloor && (!AllDoorClosed)){
				//log("------- Transition T11.4 -------------");
				newState = State.STATE_EMERGENCY;
			} else {
				//log("------- Stay in Init -------------");
				newState = State.STATE_INIT;
			}
			break;
		case STATE_GOTO_NEXT_FLOOR:
			if (curFloor != -1) {
				Target = (curFloor % Elevator.numFloors) + 1;
			}
			mDesiredFloor.set(Target, Direction.STOP, DesiredFloorHallways[Target-1]);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			
			//#transition 'T11.2'
			if ((!AllDoorClosed) && atFloor){
				//log("------- Transition T11.2 -------------");
				newState = State.STATE_GOTO_NEXT_FLOOR;
			}
			//#transition 'T11.3'
			else if (notAtFloor && (!AllDoorClosed)){
				//log("------- Transition T11.3 -------------");
				newState = State.STATE_EMERGENCY;
			} else {
				//log("------- Stay in GOTO_NEXT_FLOOR -------------");
				newState = State.STATE_GOTO_NEXT_FLOOR;
			}
			break;
		case STATE_EMERGENCY:
			//log("-----------Shouldn't be here-----------");
			Target = 1;
			mDesiredFloor.set(Target,Direction.STOP,Hallway.NONE);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			newState = State.STATE_EMERGENCY;
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
        
        state = newState;
		
        //report the current state
        setState(STATE_KEY, newState.toString());
        
        timer.start(period); 
	}
}