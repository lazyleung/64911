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
import simulator.elevatorcontrol.DoorControl;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLantern;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorOpenedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.DrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class LanternControl extends Controller{
	 public static enum State {
	        OFF,
	        ON
	    }
	 //physical message
	 private CarLanternPayload.WriteableCarLanternPayload CarLantern;
	 
	 //network messages (inputs)
	 private Utility.DoorClosedArray mDoorClosedFront;
	 private Utility.DoorClosedArray mDoorClosedBack;
	 
	 private DesiredFloorCanPayloadTranslator mDesiredFloor;
	 private Utility.AtFloorArray mAtFloor;
	 //network messages (outputs)
	 private BooleanCanPayloadTranslator mCarLantern;
	 
	 private State lanternState;
	 //TODO: remove DesiredDirection from reqs
	 //private Direction DesiredDirection;
	 private boolean verbose;
     private String name;
     
     private Direction direction;
     private Direction oppDirection;
     private Direction DesiredDirection;
     
     private SimTime period;

     public LanternControl(String name, Direction direction, SimTime period, boolean verbose){
    	 super(name + ReplicationComputer.makeReplicationString(direction),verbose);
         log("Created testlight with peroid = ",period);
    	 
         this.direction = direction;
         if(direction == Direction.UP){
        	 oppDirection = Direction.DOWN;
         }else{
        	 oppDirection = Direction.UP;
         }
         this.period = period;
         this.name = name;
         this.verbose = verbose;
         this.DesiredDirection = Direction.STOP;
         
         CarLantern = CarLanternPayload.getWriteablePayload(direction);
         physicalInterface.sendTimeTriggered(CarLantern, period);
         
         //define network objects (inputs)
     	 mDoorClosedFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
     	 mDoorClosedBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
         
     	 ReadableCanMailbox networkDesiredFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
         mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloorIn);
         canInterface.registerTimeTriggered(networkDesiredFloorIn);
     	
         mAtFloor = new Utility.AtFloorArray(canInterface);
        
        //define network objects (outputs)
         WriteableCanMailbox networkCarLanternOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_LANTERN_BASE_CAN_ID +  ReplicationComputer.computeReplicationId(direction));
         mCarLantern = new BooleanCanPayloadTranslator(networkCarLanternOut);
         mCarLantern.set(false);
         canInterface.sendTimeTriggered(networkCarLanternOut, period);
        
         lanternState = State.OFF;
         doOff();
        
         timer.start(period);
     }

     public LanternControl(Direction direction, SimTime period, boolean verbose){
         super("Lantern"+ ReplicationComputer.makeReplicationString(direction),verbose);
         log("Created testlight with peroid = ",period);
         
         this.direction = direction;
         if(direction == Direction.UP){
             oppDirection = Direction.DOWN;
         }else{
             oppDirection = Direction.UP;
         }
         this.period = period;
         this.name = name;
         this.verbose = verbose;
         this.DesiredDirection = Direction.STOP;
         
         CarLantern = CarLanternPayload.getWriteablePayload(direction);
         physicalInterface.sendTimeTriggered(CarLantern, period);
         
         //define network objects (inputs)
         mDoorClosedFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
         mDoorClosedBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
         
         ReadableCanMailbox networkDesiredFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
         mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloorIn);
         canInterface.registerTimeTriggered(networkDesiredFloorIn);
        
         mAtFloor = new Utility.AtFloorArray(canInterface);
        
        //define network objects (outputs)
         WriteableCanMailbox networkCarLanternOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_LANTERN_BASE_CAN_ID +  ReplicationComputer.computeReplicationId(direction));
         mCarLantern = new BooleanCanPayloadTranslator(networkCarLanternOut);
         mCarLantern.set(false);
         canInterface.sendTimeTriggered(networkCarLanternOut, period);
        
         lanternState = State.OFF;
         doOff();
        
         timer.start(period);
     }


	public void timerExpired(Object callbackData) {
		log("Executing LanternControl in " + lanternState);
		State newState = lanternState;
		DesiredDirection = computeDesiredDirection(mDesiredFloor.getFloor());
		log("DesiredDirectoin = " +DesiredDirection);
		switch (lanternState){
			case OFF:
				doOff();
				//#transition 'T7.2'
				if(((mDoorClosedFront.getBothClosed() && mDoorClosedBack.getBothClosed()) || mAtFloor.getCurrentFloor() == mDesiredFloor.getFloor()) && DesiredDirection == direction ){
					newState = State.ON;
				}
				break;
			case ON:
				doOn();
				//#transition 'T7.1'
				if(((mDoorClosedFront.getBothClosed() && mDoorClosedBack.getBothClosed()) || mAtFloor.getCurrentFloor() == mDesiredFloor.getFloor()) && (DesiredDirection == oppDirection || DesiredDirection == Direction.STOP) ){
					newState = State.OFF;
				}
				break;
		}
		
		if(lanternState == newState){
		    log("remains in state: ",lanternState);
		}else {
		    log("Transition:",lanternState,"->",newState);
		}

		lanternState = newState;
		setState(STATE_KEY,newState.toString());

		timer.start(period);
		
	}
	 
     
     public void doOff(){
    	//#state 'S7.1 OFF'
    	CarLantern.set(false);
    	mCarLantern.set(false);
     }
     
     public void doOn(){
    	//#state 'S7.2 ON' 
    	CarLantern.set(true);
     	mCarLantern.set(true);
     }
    
     
     public Direction computeDesiredDirection(int desiredFloor){
    	if(desiredFloor > mAtFloor.getCurrentFloor()){
    		return Direction.UP;
    	}else if(desiredFloor < mAtFloor.getCurrentFloor()){
    		return Direction.DOWN;
    	}else{
    		return Direction.STOP;
    	}
     }
     
     
     
     
     
     
     
     
}
