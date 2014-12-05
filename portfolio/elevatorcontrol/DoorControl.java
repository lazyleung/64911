//ECE649 FALL 2014
//Group 11
//Jonathan Leung/jkleung1
//Eric Newhall/enewhall
//Mengzhe Li/mzli 
//Ting Xu/tingx


package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorOpenedCanPayloadTranslator;
import simulator.elevatormodules.DoorReversalCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;

public class DoorControl extends Controller
{
    
    private enum State{
       OPEN,
       CLOSED,
       OPENING,
       CLOSING,
       NUDGE
    }

    private SimTime period;

    private double countdown;
    private State doorState;
    private boolean verbose;
  
    private final Hallway hallway;
    private final Side side;
    private int floor;
    private double dwell;
 
    //physical interface
    private WriteableDoorMotorPayload doorMotor;
    //network interface
    private DoorOpenedCanPayloadTranslator mDoorOpen;   
    private DoorClosedCanPayloadTranslator mDoorClosed;
    private DoorReversalCanPayloadTranslator mDoorReversalL;
    private DoorReversalCanPayloadTranslator mDoorReversalR;

    private DriveSpeedCanPayloadTranslator mDriveSpeed;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;  
    private CarWeightCanPayloadTranslator mCarWeight;

    private Utility.AtFloorArray    mAtFloor;

    public DoorControl(Hallway hallway, Side side, SimTime period, boolean verbose){
        super("DoorControl"+ReplicationComputer.makeReplicationString(hallway, side),verbose);
        log("Created DoorControl with peroid = ",period);

        this.hallway = hallway;
        this.side = side;
        this.period = period;
        this.verbose = verbose;
      
        this.floor = 1;
        this.countdown = 0;
        this.dwell = 5.0;
        this.doorState = State.CLOSED;

        //define physical objects
        doorMotor = DoorMotorPayload.getWriteablePayload(hallway,side);
        physicalInterface.sendTimeTriggered(doorMotor, period);

        //define network objects (inputs)
        ReadableCanMailbox networkDoorOpenIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,side));
        mDoorOpen = new DoorOpenedCanPayloadTranslator(networkDoorOpenIn,hallway,side);
        canInterface.registerTimeTriggered(networkDoorOpenIn);

        ReadableCanMailbox networkDoorClosedIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,side));
        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosedIn,hallway,side);
        canInterface.registerTimeTriggered(networkDoorClosedIn);
        
        ReadableCanMailbox networkDoorReversalLIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,Side.LEFT));
        mDoorReversalL = new DoorReversalCanPayloadTranslator(networkDoorReversalLIn,hallway,Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorReversalLIn);

        ReadableCanMailbox networkDoorReversalRIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,Side.RIGHT));
        mDoorReversalR = new DoorReversalCanPayloadTranslator(networkDoorReversalRIn,hallway,Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorReversalRIn);

        ReadableCanMailbox networkDriveSpeedIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
            mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeedIn);
            canInterface.registerTimeTriggered(networkDriveSpeedIn);

        ReadableCanMailbox networkDesiredFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
            mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloorIn);
            canInterface.registerTimeTriggered(networkDesiredFloorIn);

        ReadableCanMailbox networkCarWeightIn = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
            mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeightIn);
            canInterface.registerTimeTriggered(networkCarWeightIn);

        mAtFloor = new Utility.AtFloorArray(canInterface);

        timer.start(period);
    }
    
    public void timerExpired(Object callbackData){
        log("Executing DoorControl in " + doorState);
        State newState = doorState;

        if(mAtFloor.getCurrentFloor() != -1)
            floor = mAtFloor.getCurrentFloor();

        switch(doorState){
            case OPEN:
                doOpen();
                if(mCarWeight.getValue() < Elevator.MaxCarCapacity && countdown <= 0 ){
                    //#transition 'T5.2'
                  
                        newState = State.CLOSING;
                }
                break;
            case CLOSING:
                doClosing();
                //#transition 'T5.4'
                if(mDoorReversalL.getValue() || mDoorReversalR.getValue() || mCarWeight.getValue() >= Elevator.MaxCarCapacity){
                    newState = State.OPENING;
                //#transition 'T5.3'    
                }else if(mDoorClosed.getValue()){
                    newState = State.CLOSED;
                }
                break;
            case CLOSED:
                doClosed();
                //#transition 'T5.5'
                if(mAtFloor.isAtFloor(floor, hallway) 
                	&& (((mCarWeight.getValue() >= Elevator.MaxCarCapacity)&&(hallway==mDesiredFloor.getHallway()||mDesiredFloor.getHallway()==Hallway.BOTH)) || ((hallway==mDesiredFloor.getHallway()||mDesiredFloor.getHallway()==Hallway.BOTH) && floor==mDesiredFloor.getFloor() && mDriveSpeed.getSpeed()==0 && mDriveSpeed.getDirection() == Direction.STOP))){
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
        //setState(STATE_KEY,newState.toString());

        timer.start(period);
    
    }

    private void doOpening(){
        //#state 'S5.1 OPENING'
        doorMotor.set(DoorCommand.OPEN);
        countdown = dwell;
    }

    private void doOpen(){
        //#state 'S5.2 OPEN'
        doorMotor.set(DoorCommand.STOP);
        countdown = countdown - period.getFracSeconds();
        if(countdown < 0){
        	countdown = -1;
        }
        	
    }

    private void doClosing(){
        //#state 'S5.3 CLOSING'
        doorMotor.set(DoorCommand.CLOSE);
    }

    private void doClosed(){
        //#state 'S5.4 CLOSED'
        doorMotor.set(DoorCommand.STOP);
    }

  
}