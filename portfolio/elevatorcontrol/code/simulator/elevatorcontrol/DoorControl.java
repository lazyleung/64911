
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.*;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
import simulator.payloads.translators.IntegerCanPayloadTranslator;
import simulator.payloads.*;
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.elevatorcontrol.*;
import simulator.framework.*;

public class DoorControl extends Controller
{
    
    private enum State{
	OPEN,
	CLOSED,
	OPENING,
	CLOSING
    }

    private SimTime period;

    private double CountDown;
    private State doorState;
    private boolean verbose;
    private String name;
    
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;
 
    //physical interface
    private DoorMotorPayload.WriteableDoorMotorPayload DoorMotor;
    //network interface
    private DoorOpenedCanPayloadTranslator mDoorOpen;   
    private DoorClosedCanPayloadTranslator mDoorClosed;
    private DoorReversalCanPayloadTranslator mDoorReversal;

    private AtFloorCanPayloadTranslator mAtFloor;
    private DriveSpeedCanPayloadTranslator mDriveSpeed;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private IntegerCanPayloadTranslator mDesiredDwell;   
    private CarWeightCanPayloadTranslator mCarWeight;
    private DoorMotorCanPayloadTranslator mDoorMotor;

    public DoorControl(String name, Hallway hallway, Side side, SimTime period, boolean verbose){
	super(name + ReplicationComputer.makeReplicationString(hallway,side),verbose);
	log("Created testlight with peroid = ",period);

	//TODO: fix these
	this.floor = 1;
	this.direction = Direction.STOP;

	this.verbose = verbose;
	this.doorState = State.CLOSED;
	this.CountDown = 0;
	this.period = period;
	this.name = name;
	this.hallway = hallway;

	//define physical objects
	DoorMotor = DoorMotorPayload.getWriteablePayload(hallway,side);
	physicalInterface.sendTimeTriggered(DoorMotor,period);

	//define network objects (inputs)
	ReadableCanMailbox networkDoorOpenIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor,hallway));
	mDoorOpen = new DoorOpenedCanPayloadTranslator(networkDoorOpenIn,hallway,side);
	canInterface.registerTimeTriggered(networkDoorOpenIn);

	ReadableCanMailbox networkDoorClosedIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor,hallway));
	mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosedIn,hallway,side);
	canInterface.registerTimeTriggered(networkDoorClosedIn);
	
	ReadableCanMailbox networkDoorReversalIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor,hallway));
	mDoorReversal = new DoorReversalCanPayloadTranslator(networkDoorReversalIn,hallway,side);
	canInterface.registerTimeTriggered(networkDoorReversalIn);

	ReadableCanMailbox networkAtFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor,hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloorIn,floor,hallway);
        canInterface.registerTimeTriggered(networkAtFloorIn);

	ReadableCanMailbox networkDriveSpeedIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeedIn);
        canInterface.registerTimeTriggered(networkDriveSpeedIn);

	ReadableCanMailbox networkDesiredFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloorIn);
        canInterface.registerTimeTriggered(networkDesiredFloorIn);

	ReadableCanMailbox networkDesiredDwellIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID);
        mDesiredDwell = new IntegerCanPayloadTranslator(networkDesiredDwellIn);
        canInterface.registerTimeTriggered(networkDesiredDwellIn);

	ReadableCanMailbox networkCarWeightIn = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeightIn);
        canInterface.registerTimeTriggered(networkCarWeightIn);

	//define network objects (outputs)
	WriteableCanMailbox networkDoorMotorOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,side));
	mDoorMotor = new DoorMotorCanPayloadTranslator(networkDoorMotorOut);
	mDoorMotor.set(DoorCommand.STOP);
	canInterface.sendTimeTriggered(networkDoorMotorOut,period);

	timer.start(period);
    }

    public void timerExpired(Object callbackData){
	log("Executing DoorControl in " + doorState);
	State newState = doorState;

	switch(doorState){
	    case OPEN:
		doOpen();
		//#transition 'T5.2'
		if(CountDown <= 0){
		    newState = State.CLOSING;
		}
		break;
	    case CLOSING:
		doClosing();
		//#transition 'T5.4'
		if(mDoorReversal.getValue()){
		    newState = State.OPENING;
		//#transition 'T5.3'    
		}else if(mDoorClosed.getValue()){
		    newState = State.CLOSED;
		}
		break;
	    case CLOSED:
		doClosed();
		//#transition 'T5.5'
		if((mCarWeight.getValue() >= Elevator.MaxCarCapacity) || (mAtFloor.getValue() && floor==mDesiredFloor.getFloor() && (mDriveSpeed.getSpeed()==0 || mDriveSpeed.getDirection() == Direction.STOP))){
		    newState = State.OPENING;
		}
		break;
       	    case OPENING:
		doOpening();
		//#transition 'T5.1'
		if(mDoorOpen.getValue()){
		    newState = State.OPEN;
		}
		break;
	}

	if(doorState == newState){
	    log("remains in state: ",doorState);
	}else {
	    log("Transition:",doorState,"->",newState);
	}

	doorState = newState;
	setState(STATE_KEY,newState.toString());

	timer.start(period);
	
    }

    private void doOpening(){
	//#state 'S5.1 OPENING'
	DoorMotor.set(DoorCommand.OPEN);
	mDoorMotor.set(DoorCommand.OPEN);
	CountDown = mDesiredDwell.getValue();
    }

    private void doOpen(){
	//#state 'S5.2 OPEN'
	DoorMotor.set(DoorCommand.STOP);
	mDoorMotor.set(DoorCommand.STOP);
	CountDown = CountDown - period.getFracSeconds();
    }

    private void doClosed(){
	//#state 'S5.4 CLOSED'
	DoorMotor.set(DoorCommand.STOP);
	mDoorMotor.set(DoorCommand.STOP);
	log("doClosed()");
    }

    private void doClosing(){
	//#state 'S5.3 CLOSING'
	DoorMotor.set(DoorCommand.NUDGE);
	mDoorMotor.set(DoorCommand.NUDGE);
    }
    

}