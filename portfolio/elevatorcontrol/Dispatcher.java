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
import simulator.framework.Elevator;

public class Dispatcher extends simulator.framework.Controller{
    //store the period for the controller
    private SimTime period;
    
    //amount of time to stop as a floor.
    private final double dwellTime = 0.5;
    private final int fullCarCapacity = 13500; //Car Weight at 9 people, the car cannot let in any additional passenger therefore servicing hallcalls is useless.
    
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
    private MyBooleanCanPayloadTranslator[] mHallCalls =  new MyBooleanCanPayloadTranslator[17];
    //private HallCallCanPayloadTranslator[] mHallCalls =  new HallCallCanPayloadTranslator[17];
    private int[] HallCallFloors = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8};
    private Hallway[] HallCallHallways = {Hallway.FRONT,Hallway.BACK,Hallway.BACK,Hallway.BACK,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.FRONT,Hallway.BACK,Hallway.BACK,Hallway.FRONT};
    private Direction[] HallCallDirections = {Direction.UP,Direction.UP,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.UP,Direction.DOWN,Direction.DOWN};
    
    //receive mCarWeight message
    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;
    
    //received mCarCall message
    private ReadableCanMailbox[] networkCarCalls = new ReadableCanMailbox[10];
    private MyBooleanCanPayloadTranslator[] mCarCalls = new MyBooleanCanPayloadTranslator[10];
    private int[] CarCallFloors = {1, 1, 2, 3, 4, 5, 6, 7, 7, 8};
    private Hallway[] CarCallHallways = {Hallway.FRONT, Hallway.BACK, Hallway.BACK, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.BACK, Hallway.FRONT};
    
	
    //received mDriveSpeed message
    private DriveSpeedCanPayloadTranslator mDriveSpeed;
    
    //send mDesiredFloor messages
    private WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    //enumerate states
    private enum State {
        STATE_INIT,
        STATE_DISPATCH_UP,
        STATE_DISPATCH_DOWN,
        STATE_SERVICE_HALLCALL_UP,
        STATE_SERVICE_HALLCALL_DOWN,
        STATE_SERVICE_CARCALL_UP,
        STATE_SERVICE_CARCALL_DOWN,
        STATE_WAIT_HALLCALL_UP,
        STATE_WAIT_HALLCALL_DOWN,
        STATE_INFLIGHT_HALLCALL_UP,
        STATE_INFLIGHT_HALLCALL_DOWN,
        STATE_INFLIGHT_CARCALL_UP,
        STATE_INFLIGHT_CARCALL_DOWN,
        STATE_EMERGENCY,
        STATE_SERVICED_HALLCALL_UP,
        STATE_SERVICED_HALLCALL_DOWN,
    }
    
    private State state = State.STATE_INIT;
    private int Target;
    private Hallway DesiredHallway;
    private int curFloor = -1;
    
    private double countdown = dwellTime;
    
    private int closestFloorUp = -1;
    private int closestFloorDown = -1;
    private int closestHallCallUp = -1;
    private int farthestHallCallUp = -1;
    private int closestHallCallDown = -1;
    private int farthestHallCallDown = -1;
    private int closestCarCallUp = -1;
    private int closestCarCallDown = -1;
    
    private Hallway closestHallwayUp = Hallway.NONE;
    private Hallway farthestHallwayUp = Hallway.NONE;
    private Hallway closestHallwayDown = Hallway.NONE;
    private Hallway farthestHallwayDown = Hallway.NONE;
    
    private int HallCallDownFloor = -1;
    private int HallCallUpFloor = -1;
    
    private Hallway HallCallDownHall = Hallway.NONE;
    private Hallway HallCallUpHall = Hallway.NONE;
    
    private Direction carCallDirectionUp = Direction.STOP;
    private Direction carCallDirectionDown = Direction.STOP;
    
    private Direction DesiredDirection = Direction.STOP;
    
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
                mCarCalls[i] = new MyBooleanCanPayloadTranslator(networkCarCalls[i]);
                canInterface.registerTimeTriggered(networkCarCalls[i]);
            }
            networkHallCalls[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(HallCallFloors[i], HallCallHallways[i], HallCallDirections[i]));
            mHallCalls[i] = new MyBooleanCanPayloadTranslator(networkHallCalls[i]);
            canInterface.registerTimeTriggered(networkHallCalls[i]);
        }
    
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);
        
		ReadableCanMailbox networkCarWeightIn = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeightIn);
		canInterface.registerTimeTriggered(networkCarWeightIn);
		
        //outputs
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        Target = 1;
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
                mCarCalls[i] = new MyBooleanCanPayloadTranslator(networkCarCalls[i]);
                canInterface.registerTimeTriggered(networkCarCalls[i]);
            }
            networkHallCalls[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID+ReplicationComputer.computeReplicationId(HallCallFloors[i], HallCallHallways[i], HallCallDirections[i]));
            mHallCalls[i] = new MyBooleanCanPayloadTranslator(networkHallCalls[i]);
            canInterface.registerTimeTriggered(networkHallCalls[i]);
        }
    
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);
        
        //outputs
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        Target = 1;
        DesiredHallway = Hallway.BOTH;
        
        timer.start(period);
    }
    
    @Override
    public void timerExpired(Object callbackData) {
        State newState = state;
        
        /* Initialize the state variables */
        int closestFloor = -1;
        int farthestFloor = -1;
        
        boolean AllDoorClosed;
		if (DesiredHallway.equals(Hallway.FRONT)){
			AllDoorClosed = mDoorClosedFrontRight.getValue() && mDoorClosedFrontLeft.getValue();
		} else if (DesiredHallway.equals(Hallway.BACK)){
			AllDoorClosed =  mDoorClosedBackRight.getValue() && mDoorClosedBackLeft.getValue();
		} else {
			AllDoorClosed = mDoorClosedFrontLeft.getValue() && mDoorClosedBackLeft.getValue() && mDoorClosedFrontRight.getValue() && mDoorClosedBackRight.getValue();
		}
        boolean atFloor = false;
        
        for (int i = 0; i < mAtFloors.length; i++){
            atFloor = atFloor || mAtFloors[i].getValue();
            if (mAtFloors[i].getValue()){
                curFloor = AtFloorFloors[i];
            }
            log("AtFloor["+AtFloorFloors[i]+"]["+AtFloorHallways[i]+"] = "+mAtFloors[i].getValue());
        }
        
        boolean anyDoorOpen = !(mDoorClosedFrontLeft.getValue() && mDoorClosedBackLeft.getValue() && mDoorClosedFrontRight.getValue() && mDoorClosedBackRight.getValue());
        
        // if we are actually at a floor to make next dispatching decision.
        if ((curFloor > 0) && (mDriveSpeed.getSpeed() == 0) && (state == State.STATE_WAIT_HALLCALL_DOWN || state == State.STATE_WAIT_HALLCALL_UP || state == State.STATE_DISPATCH_UP || state == State.STATE_DISPATCH_DOWN)){
                int closestHallCall = -1;
                int closestCarCall = -1;
                int farthestHallCall = -1;
                Hallway closestHallCallHall = Hallway.NONE;
                Hallway closestCarCallHall = Hallway.NONE;
                Hallway farthestHallCallHall = Hallway.NONE;
                Hallway closestHallway = Hallway.NONE;
                carCallDirectionUp = Direction.STOP;
                
                // UP
                // search from cur floor upwards, for closest hallcall wanting to go in same direction, or farthest hallcall in opposite direction.
                for (int i = 0; i < HallCallFloors.length; i++){
                    // stop at first one found.
                    if ((HallCallFloors[i] >= curFloor) && HallCallDirections[i].equals(Direction.UP) && mHallCalls[i].getValue() && (closestHallCall == -1)){
                        
                        closestHallCall = HallCallFloors[i];
                        closestHallCallHall = HallCallHallways[i];
                        if (closestHallCall == 7){
                            if ((i+2 < HallCallFloors.length) && mHallCalls[i+2].getValue() && HallCallFloors[i+2] == 7){
                                closestHallCallHall = Hallway.BOTH;
                            }
                        }
                    }
                    log("HallCall["+HallCallFloors[i]+"]["+HallCallHallways[i]+"]["+HallCallDirections[i]+"] = "+mHallCalls[i].getValue());
                    // keep trying until the very end for a hallcall that is going down.
                    if ((HallCallFloors[i] >= curFloor) && (HallCallDirections[i].equals(Direction.DOWN)) && mHallCalls[i].getValue()){
                        farthestHallCall = HallCallFloors[i];
                        farthestHallCallHall = HallCallHallways[i];
                        // floor 7 has two possible landings. need to make desiredHall Both if necessary.
                        if ((farthestHallCall == 7) && (i+2 < HallCallFloors.length) && (HallCallFloors[i+2]==7)){
                            if (mHallCalls[i+2].getValue()){
                                farthestHallCallHall = Hallway.BOTH;
                            }
                        }
                        if ((farthestHallCall == 7) && (i-2 >= 0) && (HallCallFloors[i-2]==7)){
                            if (mHallCalls[i-2].getValue()){
                                farthestHallCallHall = Hallway.BOTH;
                            }
                        }
                    }
                }
                // search from cur floor upwards, for closest carcall.
                for (int i = 0; i < CarCallFloors.length; i++){
                    if ((CarCallFloors[i] >= curFloor) && mCarCalls[i].getValue() && (closestCarCall == -1)){
                        closestCarCall = CarCallFloors[i];
                        closestCarCallHall = CarCallHallways[i];
                        if (closestCarCall == 7){
                            if ((i+1 < CarCallFloors.length) && mCarCalls[i+1].getValue() && CarCallFloors[i+1] == 7){
                                closestCarCallHall = Hallway.BOTH;
                            }
                        }
						if (closestCarCall == 1){
                            if ((i+1 < CarCallFloors.length) && mCarCalls[i+1].getValue() && CarCallFloors[i+1] == 1){
                                closestCarCallHall = Hallway.BOTH;
                            }
                        }
                    }
                    log("Carall["+CarCallFloors[i]+"]["+CarCallHallways[i]+"] = "+mCarCalls[i].getValue());
                }
                
				if ((farthestHallCall == closestCarCall) && ((closestCarCallHall.equals(Hallway.BOTH)) || (!farthestHallCallHall.equals(closestCarCallHall)))){
					farthestHallCallHall = Hallway.BOTH;
				}
				
                for (int i = 0; i < HallCallFloors.length; i++){
                	if (i < CarCallFloors.length){
                		if ((CarCallFloors[i] > closestCarCall) && mCarCalls[i].getValue()){
                			carCallDirectionUp = Direction.UP;
                			break;
                		}
                	}
                	if ((HallCallFloors[i] > closestCarCall) && mHallCalls[i].getValue()){
                		carCallDirectionUp = Direction.UP;
                		break;
                	}
                	carCallDirectionUp = Direction.STOP;
                }
                
                //Do not service hall calls when the car is full
                if(!(mCarWeight.getValue() < fullCarCapacity)){
					closestHallCall = -1;
					farthestHallCall = -1;

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
                log("farthestHallCall= "+farthestHallCall);
                
                
                closestHallCallUp = closestHallCall;
                closestFloorUp = closestFloor;
                farthestHallCallUp = farthestHallCall;
                closestHallwayUp = closestHallway;
                farthestHallwayUp = farthestHallCallHall;
                closestCarCallUp = closestCarCall;
            
                // DOWN
                // search from cur floor downwards, for closest hallcall.
                closestHallCall = -1;
                closestCarCall = -1;
                farthestHallCall = -1;
                closestHallCallHall = Hallway.NONE;
                closestCarCallHall = Hallway.NONE;
                farthestHallCallHall = Hallway.NONE;
                closestHallway = Hallway.NONE;
                for (int i = HallCallFloors.length-1; i >= 0; i--){
                    if ((HallCallFloors[i] <= curFloor) && HallCallDirections[i].equals(Direction.DOWN) && mHallCalls[i].getValue() && (closestHallCall == -1)){
                        closestHallCall = HallCallFloors[i];
                        closestHallCallHall = HallCallHallways[i];
                        if (closestHallCall == 7){
                            // Implicit assumption that i -2 is within bounds
                            if (mHallCalls[i-2].getValue() && HallCallFloors[i-2] == 7){
                                closestHallCallHall = Hallway.BOTH;
                            }
                        }
                    }
                }
                for (int i =  HallCallFloors.length-1; i >= 0; i--){
                    if ((HallCallFloors[i] <= curFloor) && (HallCallDirections[i].equals(Direction.UP)) && mHallCalls[i].getValue()){
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
                            if ((i-1 >= 0) && (HallCallFloors[i-1] == 1) && mHallCalls[i-1].getValue()){
                                farthestHallCallHall = Hallway.BOTH;
                            }
                            if ((i+1 < HallCallFloors.length) && (HallCallFloors[i+1] == 1) && mHallCalls[i+1].getValue()){
                                farthestHallCallHall = Hallway.BOTH;
                            }
                        }
                    }
                }
                
                // search from cur floor downwards, for closest carcall.
                for (int i = CarCallFloors.length-1; i >= 0; i--){
                    if ((CarCallFloors[i] <= curFloor) && mCarCalls[i].getValue() && (closestCarCall == -1)){
                        closestCarCall = CarCallFloors[i];
                        closestCarCallHall = CarCallHallways[i];
                        if (closestCarCall == 7){
                            if ((i-1 >= 0) && mCarCalls[i-1].getValue() && (CarCallFloors[i-1] == 7)){
                                closestCarCallHall = Hallway.BOTH;
                            }
                        } else if (closestCarCall == 1){
                            if ((i-1 >= 0) && mCarCalls[i-1].getValue() && CarCallFloors[i-1] == 1){
                                closestCarCallHall = Hallway.BOTH;
                            }
                        }
                    }
                }
                
                for (int i = 0; i < HallCallFloors.length; i++){
                	if (i < CarCallFloors.length){
                		if ((CarCallFloors[i] < closestCarCall) && mCarCalls[i].getValue()){
                			carCallDirectionDown = Direction.DOWN;
                			break;
                		}
                	}
                	if ((HallCallFloors[i] < closestCarCall) && mHallCalls[i].getValue()){
                		carCallDirectionDown = Direction.DOWN;
                		break;
                	}
                	carCallDirectionDown = Direction.STOP;
                }
                
				if ((farthestHallCall == closestCarCall) && ((closestCarCallHall.equals(Hallway.BOTH)) || (!farthestHallCallHall.equals(closestCarCallHall)))){
					farthestHallCallHall = Hallway.BOTH;
				}
                
                //Do not service hall calls when the car is full
				if(!(mCarWeight.getValue() < fullCarCapacity)){
					closestHallCall = -1;
					farthestHallCall = -1;
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
                log("farthestHallCall= "+farthestHallCall);
                
                
                closestHallCallDown = closestHallCall;
                closestFloorDown = closestFloor;
                farthestHallCallDown = farthestHallCall;
                closestHallwayDown = closestHallway;
                farthestHallwayDown = farthestHallCallHall;
                closestCarCallDown = closestCarCall;
        }
		
		
        switch (state){
        case STATE_INIT:
            Target = 1;
            DesiredDirection = Direction.STOP;
            countdown = dwellTime;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);

            //#transition 'T11.24'
            if ((!atFloor) && anyDoorOpen){
                newState = State.STATE_EMERGENCY;
            }
            //#transition 'T11.1'
            else {
                newState = State.STATE_DISPATCH_UP;
            }
            break;
        case STATE_DISPATCH_UP:
        	countdown = dwellTime;
        	HallCallUpFloor = closestHallCallUp;
        	HallCallUpHall = closestHallwayUp;
        	HallCallDownFloor = farthestHallCallUp;
        	HallCallDownHall = farthestHallwayUp;
            DesiredHallway = Hallway.NONE;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
            //#transition 'T11.3'
            if ((closestFloorUp == -1) && (farthestHallCallUp == -1)){
                newState = State.STATE_DISPATCH_DOWN;
            }
            //#transition 'T11.4'
            else if ((closestFloorUp != -1) && (closestFloorUp == closestHallCallUp) && (mCarWeight.getValue() < fullCarCapacity)) {
                newState = State.STATE_SERVICE_HALLCALL_UP;
            }
            //#transition 'T11.12'
            else if (((closestFloorUp == -1) && (farthestHallCallUp != -1) && (mCarWeight.getValue() < fullCarCapacity)) || ((closestFloorUp != -1) && (farthestHallCallUp != -1) && (closestFloorUp == farthestHallCallUp) && (  carCallDirectionUp != Direction.UP  ))){
                newState = State.STATE_SERVICE_HALLCALL_DOWN;
            } 
            //#transition 'T11.14'
            else if ((closestFloorUp != -1) && (closestFloorUp != closestHallCallUp)) {
                newState = State.STATE_SERVICE_CARCALL_UP;
            }
            else {
                newState = State.STATE_DISPATCH_UP;
            }
            break;
        case STATE_DISPATCH_DOWN:
        	countdown = dwellTime;
        	HallCallDownFloor = closestHallCallDown;
        	HallCallDownHall = closestHallwayDown;
        	HallCallUpFloor = farthestHallCallDown;
        	HallCallUpHall = farthestHallwayDown;
            DesiredHallway = Hallway.NONE;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
            //#transition 'T11.2'
            if ((closestFloorDown == -1) && (farthestHallCallDown == -1)){
                newState = State.STATE_DISPATCH_UP;
            }
            //#transition 'T11.8'
            else if ((closestFloorDown != -1) && (closestFloorDown == closestHallCallDown) && (mCarWeight.getValue() < fullCarCapacity)) {
                newState = State.STATE_SERVICE_HALLCALL_DOWN;
            }
            //#transition 'T11.13'
            else if (((closestFloorDown == -1) && (farthestHallCallDown != -1) && (mCarWeight.getValue() < fullCarCapacity)) || ((closestFloorDown != -1) && (farthestHallCallDown != -1) && (closestFloorDown == farthestHallCallDown) && (carCallDirectionDown != Direction.DOWN))) {
                newState = State.STATE_SERVICE_HALLCALL_UP;
            }
            //#transition 'T11.17'
            else if ((closestFloorDown != -1) && (closestHallCallDown != closestFloorDown)) {
                newState = State.STATE_SERVICE_CARCALL_DOWN;
            } else {
                newState = State.STATE_DISPATCH_DOWN;
            }
            break;
        case STATE_SERVICE_HALLCALL_UP:
        	countdown = dwellTime;
            Target = HallCallUpFloor; 
            DesiredDirection = Direction.UP;
            DesiredHallway = HallCallUpHall; 
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
    
            //#transition 'T11.5'
            newState = State.STATE_INFLIGHT_HALLCALL_UP;
            break;
        case STATE_SERVICE_HALLCALL_DOWN:
        	countdown = dwellTime;
            Target = HallCallDownFloor; 
            DesiredDirection = Direction.DOWN;
            DesiredHallway = HallCallDownHall; 
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
            
            //#trnasition 'T11.9'
            newState = State.STATE_INFLIGHT_HALLCALL_DOWN;
            break;
        case STATE_INFLIGHT_HALLCALL_UP:
        	countdown = dwellTime;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
                
            //#transition 'T11.6'
            if ((!AllDoorClosed) && (Target == curFloor) && (atFloor)) {
                newState = State.STATE_SERVICED_HALLCALL_UP;
            }
            //#transition 'T11.21'
            else if ((!atFloor) && anyDoorOpen){
                newState = State.STATE_EMERGENCY;
            }
            else {
                newState = State.STATE_INFLIGHT_HALLCALL_UP;
            }
            break;
        case STATE_INFLIGHT_HALLCALL_DOWN:
        	countdown = dwellTime;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
            
            //#transition 'T11.10'
            if ((!AllDoorClosed) && (Target == curFloor) && (atFloor)) {
                newState = State.STATE_SERVICED_HALLCALL_DOWN;
            }
            //#transition 'T11.22'
            else if ((!atFloor) && anyDoorOpen){
                newState = State.STATE_EMERGENCY;
            }
            else {
                newState = State.STATE_INFLIGHT_HALLCALL_DOWN;
            }
            break;
        case STATE_SERVICED_HALLCALL_UP:
            countdown = dwellTime;
            DesiredHallway = Hallway.NONE;
            mDesiredFloor.set(Target,DesiredDirection,DesiredHallway);
                
            //#transition 'T11.25'
            if (AllDoorClosed){
                newState = State.STATE_WAIT_HALLCALL_UP;
            } else {
                newState = State.STATE_SERVICED_HALLCALL_UP;
            }
            break;
        case STATE_SERVICED_HALLCALL_DOWN:
            countdown = dwellTime;
            DesiredHallway = Hallway.NONE;
            mDesiredFloor.set(Target,DesiredDirection,DesiredHallway);
            
            //#transition 'T11.26'
            if (AllDoorClosed){
                newState = State.STATE_WAIT_HALLCALL_DOWN;
            } else {
                newState = State.STATE_SERVICED_HALLCALL_DOWN;
            }
            break;
        case STATE_WAIT_HALLCALL_UP:
            countdown = countdown - period.getFracSeconds();
            if (countdown < 0){
                countdown = -1;
            }
        	mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
        	
        	//#transition 'T11.7'
        	if (countdown <= 0){
				newState = State.STATE_DISPATCH_UP;
        	} else {
        		newState = State.STATE_WAIT_HALLCALL_UP;
        	}
            break;
        case STATE_WAIT_HALLCALL_DOWN:
            countdown = countdown - period.getFracSeconds();
            if (countdown < 0){
                countdown = -1;
            }
        	mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
        	
        	//#transition 'T11.11'
        	if (countdown <= 0){
				newState = State.STATE_DISPATCH_DOWN;
        	} else {
        		newState = State.STATE_WAIT_HALLCALL_DOWN;
        	}
            break;
        case STATE_SERVICE_CARCALL_UP:
        	countdown = dwellTime;
            Target = closestFloorUp;
        	DesiredDirection = carCallDirectionUp;
        	DesiredHallway = closestHallwayUp;
        	mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
        	
            //#transition 'T11.15'
        	newState = State.STATE_INFLIGHT_CARCALL_UP;
        	break;
        case STATE_INFLIGHT_CARCALL_UP:
        	countdown = dwellTime;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
                
            //#transition 'T11.16'
            if ((!AllDoorClosed) && (Target == curFloor) && (atFloor)) {
                newState = State.STATE_DISPATCH_UP;
            }
            //#transition 'T11.20'
            else if ((!atFloor) && anyDoorOpen){
                newState = State.STATE_EMERGENCY;
            }
            else {
                newState = State.STATE_INFLIGHT_CARCALL_UP;
            }
            break;
        case STATE_SERVICE_CARCALL_DOWN:
        	countdown = dwellTime;
        	Target = closestFloorDown;
        	DesiredDirection = carCallDirectionDown;
        	DesiredHallway = closestHallwayDown;
        	mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
        	
            //#transition 'T11.18'
        	newState = State.STATE_INFLIGHT_CARCALL_DOWN;
        	break;
        case STATE_INFLIGHT_CARCALL_DOWN:
        	countdown = dwellTime;
            mDesiredFloor.set(Target, DesiredDirection, DesiredHallway);
                
            //#transition 'T11.19'
            if ((!AllDoorClosed) && (Target == curFloor) && (atFloor)) {
                newState = State.STATE_DISPATCH_DOWN;
            }
            //#transition 'T11.23'
            else if ((!atFloor) && anyDoorOpen){
                newState = State.STATE_EMERGENCY;
            }
            else {
                newState = State.STATE_INFLIGHT_CARCALL_DOWN;
            }
            break;
        case STATE_EMERGENCY:
            //log("-----------Shouldn't be here-----------");
            Target = 1;
            DesiredHallway = Hallway.NONE;
            mDesiredFloor.set(Target,Direction.STOP,Hallway.NONE);

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
        log("mDesiredFloor f = "+Target+" Hall="+DesiredHallway);
        state = newState;
        //System.out.println(state);
        //report the current state
        setState(STATE_KEY, newState.toString());
        
        timer.start(period); 
    }
}