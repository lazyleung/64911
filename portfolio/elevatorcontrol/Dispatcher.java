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
    private Hallway[] HallCallHallways = {Hallway.FRONT,Hallway.BACK,Hallway.BACK,Hallway.BACK,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.BACK,Hallway.BACK,Hallway.FRONT};
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
    
    
    //received mDriveSpeed message
    private DriveSpeedCanPayloadTranslator mDriveSpeed;
    
    //send mDesiredFloor messages
    private WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    //send mDesiredDwell messages
    private WriteableCanMailbox networkDesiredDwellFront;
    private WriteableCanMailbox networkDesiredDwellBack;
    private DesiredDwellCanPayloadTranslator mDesiredDwellFront;
    private DesiredDwellCanPayloadTranslator mDesiredDwellBack;
	
    //enumerate states
    private enum State {
        STATE_INIT,
        STATE_GOTO_CLOSEST_FLOOR,
        STATE_GOTO_FARTHEST_FLOOR,
        STATE_EMERGENCY,
		STATE_IDLE,
    }
    
    private State state = State.STATE_INIT;
    private int Target;
	private Hallway DesiredHallway;
    private int curFloor = -1;
    private int closestFloor = -1;
    private int farthestFloor = -1;
    private Hallway closestHallway = Hallway.NONE;
    private Hallway farthestHallway = Hallway.NONE;
    private Direction currentDirection = Direction.STOP;
    private Direction oppositeDirection = Direction.STOP;
    
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
	
        ReadableCanMailbox networkDriveSpeedIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeedIn);
        canInterface.registerTimeTriggered(networkDriveSpeedIn);
        
        
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
        
        Target = -1;
        DesiredHallway = Hallway.BOTH;
        
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
	
        ReadableCanMailbox networkDriveSpeedIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeedIn);
        canInterface.registerTimeTriggered(networkDriveSpeedIn);
        
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
        
        Target = -1;
        DesiredHallway = Hallway.BOTH;
        
        timer.start(period);
	}

	
	private Direction getOppositeDirection(Direction d){
		if (Direction.UP.equals(d)){
			return Direction.DOWN;
		} else if (Direction.DOWN.equals(d)){
			return Direction.UP;
		} else {
			return Direction.STOP;
		}
	}
	
	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		
		/* Initialize the state variables */
		
		
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
		log("-=-----------CURFLOR  "+curFloor+ "    Target="+Target+"------------------");
		
		if (curFloor == 1){
			currentDirection = Direction.UP; // you can only go up.
		}
		if (curFloor == 8){
			currentDirection = Direction.DOWN; // you can only go down.
		}
		if (notAtFloor){
			currentDirection = mDriveSpeed.getDirection(); // get direction from mDriveSpeed
		}
		oppositeDirection = getOppositeDirection(currentDirection);
		
		// if we are actually at a floor to make next dispatching decision.
		if (curFloor > 0){
			if (currentDirection.equals(Direction.UP)){
				int closestHallCall = -1;
				int closestCarCall = -1;
				int farthestHallCall = -1;
				Hallway closestHallCallHall = Hallway.NONE;
				Hallway closestCarCallHall = Hallway.NONE;
				Hallway farthestHallCallHall = Hallway.NONE;
				// search from cur floor upwards, for closest hallcall wanting to go in same direction, or farthest hallcall in opposite direction.
				for (int i = 0; i < HallCallFloors.length; i++){
					// stop at first one found.
					if ((HallCallFloors[i] > curFloor) && HallCallDirections[i].equals(Direction.UP) && mHallCalls[i].getValue() && (closestHallCall == -1)){
						closestHallCall = HallCallFloors[i];
						closestHallCallHall = HallCallHallways[i];
						if (closestHallCall == 7){
							if (mHallCalls[i+2].getValue()){
								closestHallCallHall = Hallway.BOTH;
							}
						}
					}
					// keep trying until the very end for a hallcall that is going down.
					if ((HallCallFloors[i] > curFloor) && (HallCallDirections[i].equals(oppositeDirection)) && mHallCalls[i].getValue()){
						farthestHallCall = HallCallFloors[i];
						farthestHallCallHall = HallCallHallways[i];
						
						// floor 7 has two possible landings. need to make desiredHall Both if necessary.
						if ((farthestHallCall == 7) && (HallCallFloors[i+2]==7)){
							if (mHallCalls[i+2].getValue()){
								farthestHallCallHall = Hallway.BOTH;
							}
						}
						if ((farthestHallCall == 7) && (HallCallFloors[i-2]==7)){
							if (mHallCalls[i-2].getValue()){
								farthestHallCallHall = Hallway.BOTH;
							}
						}
					}
				}
				// search from cur floor upwards, for closest carcall.
				for (int i = 0; i < CarCallFloors.length; i++){
					if ((CarCallFloors[i] > curFloor) && mCarCalls[i].getValue() && (closestCarCall == -1)){
						closestCarCall = CarCallFloors[i];
						closestCarCallHall = CarCallHallways[i];
						if (closestCarCall == 7){
							if (mCarCalls[i+1].getValue()){
								closestCarCallHall = Hallway.BOTH;
							}
						}
					}
				}
				
				// set the closest Floor and Hallway based on carcalls and hallcalls
				if (closestHallCall == -1){  // no closestHall
					closestFloor = closestCarCall;
					closestHallway = closestCarCallHall;
				} else if (closestCarCall == -1) { // no closestCar
					closestFloor = closestHallCall;
					closestHallway = closestHallCallHall;
				} else {
					if (closestHallCall < closestCarCall){
						closestFloor = closestHallCall;
						closestHallway = closestHallCallHall;
					} else if (closestCarCall < closestHallCall){
						closestFloor = closestCarCall;
						closestHallway = closestCarCallHall;
					} else {
						closestFloor = closestHallCall;
						if (closestCarCallHall.equals(closestHallCallHall)){
							closestHallway = closestHallCallHall;
						} else {
							if ((closestHallCallHall != Hallway.NONE) && (closestCarCallHall != Hallway.NONE)){
								closestHallway = Hallway.BOTH;
							} else if (closestHallCallHall != Hallway.NONE) {
								// should never be here
								log("-------------------SHOULD NOT BE HERE----------------");
								closestHallway = closestHallCallHall;
							} else {
								// should never be here
								log("-------------------SHOULD NOT BE HERE----------------");
								closestHallway = closestCarCallHall;
							}
						}
					}
				}
				farthestFloor = farthestHallCall;
				farthestHallway = farthestHallCallHall;
			}
			
			else if (currentDirection.equals(Direction.DOWN)){
				// search from cur floor downwards, for closest hallcall.
				int closestHallCall = -1;
				int closestCarCall = -1;
				int farthestHallCall = -1;
				Hallway closestHallCallHall = Hallway.NONE;
				Hallway closestCarCallHall = Hallway.NONE;
				Hallway farthestHallCallHall = Hallway.NONE;
				for (int i = HallCallFloors.length-1; i >= 0; i--){
					if ((HallCallFloors[i] < curFloor) && HallCallDirections[i].equals(Direction.DOWN) && mHallCalls[i].getValue() && (closestHallCall == -1)){
						closestHallCall = HallCallFloors[i];
						closestHallCallHall = HallCallHallways[i];
						if (closestHallCall == 7){
							if (mHallCalls[i-2].getValue()){
								closestHallCallHall = Hallway.BOTH;
							}
						}
					}
				}
				for (int i=  HallCallFloors.length-1; i >= 0; i--){
					if ((HallCallFloors[i] < curFloor) && (HallCallDirections[i].equals(Direction.UP)) && mHallCalls[i].getValue()){
						farthestHallCall = HallCallFloors[i];
						farthestHallCallHall = HallCallHallways[i];
						if (farthestHallCall == 7){
							if ((HallCallFloors[i-2] == 7) && mHallCalls[i-2].getValue()){
								farthestHallCallHall = Hallway.BOTH;
							}
							if ((HallCallFloors[i+2] == 7) && mHallCalls[i+2].getValue()){
								farthestHallCallHall = Hallway.BOTH;
							}
						}
						if (farthestHallCall == 1){
							if ((HallCallFloors[i-1] == 1) && mHallCalls[i-1].getValue()){
								farthestHallCallHall = Hallway.BOTH;
							}
							if ((HallCallFloors[i+1] == 1) && mHallCalls[i+1].getValue()){
								farthestHallCallHall = Hallway.BOTH;
							}
						}
					}
				}
				
				// search from cur floor downwards, for closest carcall.
				for (int i = CarCallFloors.length-1; i >= 0; i--){
					if ((CarCallFloors[i] < curFloor) && mCarCalls[i].getValue() && (closestCarCall == -1)){
						closestCarCall = CarCallFloors[i];
						closestCarCallHall = CarCallHallways[i];
						if (closestCarCall == 7){
							if (mCarCalls[i-1].getValue()){
								closestCarCallHall = Hallway.BOTH;
							}
						} else if (closestCarCall == 1){
							if (mCarCalls[i-1].getValue()){
								closestCarCallHall = Hallway.BOTH;
							}
						}
					}
				}
				
				// set the closest Floor and Hallway based on carcalls and hallcalls
				if (closestHallCall == -1){
					closestFloor = closestCarCall;
					closestHallway = closestCarCallHall;
				} else if (closestCarCall == -1) {
					closestFloor = closestHallCall;
					closestHallway = closestHallCallHall;
				} else {
					if (closestHallCall > closestCarCall){
						closestFloor = closestHallCall;
						closestHallway = closestHallCallHall;
					} else if (closestCarCall > closestHallCall){
						closestFloor = closestCarCall;
						closestHallway = closestCarCallHall;
					} else {
						closestFloor = closestHallCall;
						if (closestCarCallHall.equals(closestHallCallHall)){
							closestHallway = closestHallCallHall;
						} else {
							if ((closestHallCallHall != Hallway.NONE) && (closestCarCallHall != Hallway.NONE)){
								closestHallway = Hallway.BOTH;
							} else if (closestHallCallHall != Hallway.NONE) {
								// should never be here
								log("-------------------SHOULD NOT BE HERE----------------");
								closestHallway = closestHallCallHall;
							} else {
								// should never be here
								log("-------------------SHOULD NOT BE HERE----------------");
								closestHallway = closestCarCallHall;
							}
						}
					}
				}
				farthestFloor = farthestHallCall;
				farthestHallway = farthestHallCallHall;
			}
		}
		
        if ((closestFloor == -1) && (farthestFloor == -1)){
            currentDirection = oppositeDirection;
            oppositeDirection = currentDirection;
        }
		
		
		log("curFloor="+curFloor+" atFloor="+atFloor+" AllDoorClosed="+AllDoorClosed + " ClosestFloor="+closestFloor + "  closestHall="+closestHallway + " farthestFloor="+farthestFloor+ "  farthestHall="+farthestHallway+" curDir="+currentDirection);
		oppositeDirection = getOppositeDirection(currentDirection);
		
		log("curFloor="+curFloor+"  Target="+Target + "   atFloor="+atFloor+"    allDoorClosed="+AllDoorClosed);
		
		switch (state){
		case STATE_INIT:
			Target = 1;
			DesiredHallway = Hallway.BOTH;
			mDesiredFloor.set(Target, Direction.STOP, DesiredHallway);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			
			//#transition 'T11.1'
			if ((!AllDoorClosed) && atFloor && (curFloor == Target) && (closestFloor != -1)){
				//log("------- Transition T11.1 -------------");
				newState = State.STATE_GOTO_CLOSEST_FLOOR;
			}
			//#transition 'T11.2'
			else if ((!AllDoorClosed) && atFloor && (curFloor == Target) && (closestFloor == -1) && (farthestFloor != -1)){
				newState = State.STATE_GOTO_FARTHEST_FLOOR;
			}
			//#transition 'T11.7'
			else if (notAtFloor && (!AllDoorClosed)){
				newState = State.STATE_EMERGENCY;
			} else {
				newState = State.STATE_INIT;
			}
			break;
		case STATE_GOTO_CLOSEST_FLOOR:
			Target = closestFloor;
			DesiredHallway = closestHallway;
			mDesiredFloor.set(Target, Direction.STOP, closestHallway);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			
            //#transition 'T11.3'
			newState = State.STATE_IDLE;
			break;
		case STATE_GOTO_FARTHEST_FLOOR:
			Target = farthestFloor;
			DesiredHallway = farthestHallway;
			mDesiredFloor.set(Target, Direction.STOP, farthestHallway);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			
			//transition 'T11.4'
			newState = State.STATE_IDLE;
			break;
		case STATE_EMERGENCY:
			//log("-----------Shouldn't be here-----------");
			Target = 1;
			DesiredHallway = Hallway.NONE;
			mDesiredFloor.set(Target,Direction.STOP,Hallway.NONE);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
			newState = State.STATE_EMERGENCY;
			break;
		case STATE_IDLE:
			mDesiredFloor.set(Target, Direction.STOP, DesiredHallway);
			mDesiredDwellFront.set(dwellTime);
			mDesiredDwellBack.set(dwellTime);
            //#transition 'T11.8'
			if (notAtFloor && (!AllDoorClosed)){
				newState = State.STATE_EMERGENCY;
			} 
			//#transition 'T11.5'
			else if ((!AllDoorClosed) && (curFloor == Target) && atFloor && (closestFloor != -1)){
				newState = State.STATE_GOTO_CLOSEST_FLOOR;
			} 
			//#transition 'T11.6'
			else if ((!AllDoorClosed) && (curFloor == Target) && atFloor && (closestFloor == -1) && (farthestFloor != -1)){
				newState = State.STATE_GOTO_FARTHEST_FLOOR;
			}
			else {
				newState = State.STATE_IDLE;
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
        log("mDesiredFloor f = "+Target+" Hall="+DesiredHallway);
        state = newState;
		
        //report the current state
        setState(STATE_KEY, newState.toString());
        
        timer.start(period); 
	}
}