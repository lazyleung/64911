package simulator.elevatorcontrol;

import simulator.elevatormodules.DriveObject;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload.ReadableHallLightPayload;

public class RuntimeRequirementsMonitor extends RuntimeMonitor {

	RT6StateMachine RT6State = new RT6StateMachine();
	RT7StateMachine RT7State = new RT7StateMachine();
	RT8_1StateMachine RT8_1State = new RT8_1StateMachine();
	RT8_2StateMachine RT8_2State = new RT8_2StateMachine();
	RT8_3StateMachine RT8_3State = new RT8_3StateMachine();
	RT9StateMachine RT9State = new RT9StateMachine();
	RT10StateMachine RT10State = new RT10StateMachine();

	protected boolean[][] mCarCalls = new boolean[Elevator.numFloors][2];
	protected boolean[][][] mHallCalls = new boolean[Elevator.numFloors][2][2];
	protected boolean[][] carCalls = new boolean[Elevator.numFloors][2];
	protected boolean[][] hallCalls = new boolean[Elevator.numFloors][4];
	protected boolean upCarLantern = false;
	protected boolean downCarLantern = false;
	protected boolean[] DoorOpen = new boolean[4];
	protected int currentFloor = MessageDictionary.NONE;
	protected Hallway currentHallway = Hallway.NONE;
	protected boolean reversal[][] = new boolean[2][2];

	@Override
	public void timerExpired(Object callbackData) {
		// do nothing
	}

	@Override
	protected String[] summarize() {
		// no outputs
		String[] arr = new String[1];
		return arr;
	}

	public void receive(ReadableCarCallPayload msg) {
		if (msg.pressed()) {
			int floor = msg.getFloor();
			mCarCalls[floor - 1][msg.getHallway().ordinal()] = true;
		}
	}

	public void receive(ReadableHallCallPayload msg) {
		if (msg.pressed()) {
			int floor = msg.getFloor();
			mHallCalls[floor - 1][msg.getHallway().ordinal()][msg
					.getDirection().ordinal()] = true;
			//warning("Hallcall: floor "+floor+"Hallway "+msg.getHallway());
		}
	}

	public void receive(ReadableHallLightPayload msg) {
		if (msg.getDirection() == Direction.UP) {
			if (msg.getHallway() == Hallway.FRONT) {
				hallCalls[msg.getFloor() - 1][0] = msg.lighted();
			} else {
				hallCalls[msg.getFloor() - 1][1] = msg.lighted();
			}
		} else {
			if (msg.getHallway() == Hallway.FRONT) {
				hallCalls[msg.getFloor() - 1][2] = msg.lighted();
			} else {
				hallCalls[msg.getFloor() - 1][3] = msg.lighted();
			}
		}
	}

	public void receive(ReadableCarLanternPayload msg) {
		updateCarLanterns(msg);
		RT8_2State.receive(msg);
	}

	public void receive(ReadableAtFloorPayload msg) {
		updateCurrentFloor(msg);
	}

	public void receive(ReadableDoorReversalPayload msg) {
		setReversal(msg);
	}

	public void receive(ReadableDriveSpeedPayload msg) {
		RT6State.receive(msg);
		RT9State.receive(msg);
		RT8_3State.receive(msg);
		unsetReversal(msg);
	}

	public void receive(ReadableDoorClosedPayload msg) {
		RT7State.receive(msg);
	}

	public void receive(ReadableDoorOpenPayload msg) {
		if(msg.getHallway() == Hallway.FRONT && msg.getSide() == Side.LEFT){
			DoorOpen[0] = msg.isOpen();
		}else if(msg.getHallway() == Hallway.FRONT && msg.getSide() == Side.RIGHT){
			DoorOpen[1] = msg.isOpen();
		}else if(msg.getHallway() == Hallway.BACK && msg.getSide() == Side.LEFT){
			DoorOpen[2] = msg.isOpen();
		}else{
			DoorOpen[3] = msg.isOpen();
		}
		RT8_1State.receive(msg);
		mCarCalls[currentFloor-1][msg.getHallway().ordinal()]=carLights[currentFloor-1][msg.getHallway().ordinal()].lighted();
		mHallCalls[currentFloor-1][msg.getHallway().ordinal()][0]=hallLights[currentFloor-1][msg.getHallway().ordinal()][0].lighted();
		mHallCalls[currentFloor-1][msg.getHallway().ordinal()][1]=hallLights[currentFloor-1][msg.getHallway().ordinal()][1].lighted();
		//warning("Floor served: floor "+currentFloor+" hallway: "+msg.getHallway());


	}

	public void receive(ReadableDoorMotorPayload msg) {
		RT10State.receive(msg);
	}

	private void updateCarLanterns(ReadableCarLanternPayload msg) {
		if (msg.getDirection() == Direction.UP) {
			if (msg.lighted()) {
				upCarLantern = true;
			} else {
				upCarLantern = false;
			}
		} else if (msg.getDirection() == Direction.DOWN) {
			if (msg.lighted()) {
				downCarLantern = true;
			} else {
				downCarLantern = false;
			}
		}
	}

	private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
		if (lastAtFloor.getFloor() == currentFloor) {
			// the atFloor message is for the currentfloor, so check both sides
			// to see if they a
			if (!atFloors[lastAtFloor.getFloor() - 1][Hallway.BACK.ordinal()]
					.value()
					&& !atFloors[lastAtFloor.getFloor() - 1][Hallway.FRONT
							.ordinal()].value()) {
				// both sides are false, so set to NONE
				currentFloor = MessageDictionary.NONE;
				currentHallway = Hallway.NONE;
			}
			// otherwise at least one side is true, so leave the current floor
			// as is
		} else {
			if (lastAtFloor.value()) {
				currentFloor = lastAtFloor.getFloor();
				currentHallway = lastAtFloor.getHallway();
			}
		}
	}

	private void setReversal(ReadableDoorReversalPayload msg) {
		if (msg.isReversing()) {
			reversal[msg.getHallway().ordinal()][msg.getSide().ordinal()] = true;
		}
	}

	private void unsetReversal(ReadableDriveSpeedPayload msg) {
		if (msg.speed() > DriveObject.LevelingSpeed) {
			reversal[Hallway.BACK.ordinal()][Side.LEFT.ordinal()] = false;
			reversal[Hallway.FRONT.ordinal()][Side.LEFT.ordinal()] = false;
			reversal[Hallway.BACK.ordinal()][Side.RIGHT.ordinal()] = false;
			reversal[Hallway.FRONT.ordinal()][Side.RIGHT.ordinal()] = false;
		}
	}

	private static enum RT6States {
		MOVING, STOPPED, STOPPED_NO_PENDING_CALLS
	}

	private static enum RT7States {
		DOORS_CLOSED, DOORS_OPEN, DOORS_OPEN_NO_PENDING_CALLS
	}

	private static enum RT8_1States {
		DOORS_CLOSED, OPEN_ON, OPEN_OFF
	}

	private static enum RT8_2States {
		LANTERNS_OFF, LANTERNS_ON, LANTERNS_CHANGED
	}

	private static enum RT8_3States {
		IDLE, WRONG_DIRECTION
	}

	private static enum RT9States {
		STOPPED_FAST, STOPPED_NO_FAST, MOVING
	}

	private static enum RT10States {
		DOORS_STOPPED, DOOR_NUDGING_AFTER_REVERSAL, DOORS_NUDGING_NO_REVERSAL
	}

	private class RT6StateMachine {
		RT6States state;
		boolean warningIssued = false;

		public RT6StateMachine() {
			state = RT6States.STOPPED;
		}

		public void receive(ReadableDriveSpeedPayload msg) {
			updateState(msg);
		}

		private void updateState(ReadableDriveSpeedPayload msg) {
			RT6States previousState = state;
			RT6States newState = previousState;
			
			switch(state){
			
			case MOVING:
				if (msg.speed() == 0
				&& currentFloor > 0
				&& currentFloor <= 8
				&& (mCarCalls[currentFloor - 1][0]
						|| mCarCalls[currentFloor - 1][1]
						|| mHallCalls[currentFloor - 1][0][0]
						|| mHallCalls[currentFloor - 1][1][0]
						|| mHallCalls[currentFloor - 1][0][1] 
						|| mHallCalls[currentFloor - 1][1][1])) {
					newState = RT6States.STOPPED;
				} else if (msg.speed() == 0
						&& currentFloor > 0
						&& currentFloor <= 8
						&& (!mCarCalls[currentFloor - 1][0]
								&& !mCarCalls[currentFloor - 1][1]
								&& !mHallCalls[currentFloor - 1][0][0]
								&& !mHallCalls[currentFloor - 1][1][0]
								&& !mHallCalls[currentFloor - 1][0][1]
								&& !mHallCalls[currentFloor - 1][1][1])) {
					newState = RT6States.STOPPED_NO_PENDING_CALLS;
				}
				break;
			case STOPPED:
			case STOPPED_NO_PENDING_CALLS:
				if (msg.speed() > DriveObject.LevelingSpeed) {
					newState = RT6States.MOVING;
				}
				break;			
			}

			if (newState != previousState) {
				switch (newState) {
				case MOVING:
					warningIssued = false;
					break;
				case STOPPED:
					break;
				case STOPPED_NO_PENDING_CALLS:
					if (!warningIssued) {
						warning("R-T.6 Violated: Car stopped at Floor "
								+ currentFloor
								+ " when there is no pending calls");
						warningIssued = true;
					}
					break;

				}
			}

			state = newState;

		}
	}

	private class RT7StateMachine {
		RT7States state;

		public RT7StateMachine() {
			state = RT7States.DOORS_CLOSED;
		}

		public void receive(ReadableDoorClosedPayload msg) {
			if (!msg.isClosed()) {
				// warning("door open!");
			}
			updateState(msg);

		}

		private void updateState(ReadableDoorClosedPayload msg) {
			RT7States previousState = state;
			RT7States newState = previousState;
			
			switch(state){
			case DOORS_CLOSED:
				if (!msg.isClosed()
						&& currentFloor > 0
						&& currentFloor <= 8
						&& (mCarCalls[currentFloor - 1][msg.getHallway().ordinal()]
								|| mHallCalls[currentFloor - 1][msg.getHallway().ordinal()][0]
								|| mHallCalls[currentFloor - 1][msg.getHallway().ordinal()][1])) {
					// For now direction does not matter. as long as there is a hall
					// call placed the car can stop at that hallway.
					newState = RT7States.DOORS_OPEN;
				} else if (!msg.isClosed()
						&& currentFloor > 0
						&& currentFloor <= 8
						&& (!mCarCalls[currentFloor - 1][msg.getHallway().ordinal()]
								&& !mHallCalls[currentFloor - 1][msg.getHallway().ordinal()][0]
								&& !mHallCalls[currentFloor - 1][msg.getHallway().ordinal()][1])) {
					// For now direction does not matter. as long as there is a hall
					// call placed the car can stop at that hallway.
					newState = RT7States.DOORS_OPEN_NO_PENDING_CALLS;
				}
				break;
			case DOORS_OPEN:
			case DOORS_OPEN_NO_PENDING_CALLS:
				if (msg.isClosed()) {
					newState = RT7States.DOORS_CLOSED;
				}
				break;

			}

			if (newState != previousState) {
				switch (newState) {
				case DOORS_CLOSED:
					break;
				case DOORS_OPEN:
					break;
				case DOORS_OPEN_NO_PENDING_CALLS:
					warning("R-T.7 Violated: Car doors open at Floor "
							+ currentFloor + " " + msg.getHallway()
							+ " when there is no pending calls");
					break;
				}
			}
			state = newState;

		}
	}

	private class RT10StateMachine {
		RT10States state[][] = new RT10States[2][2];

		public RT10StateMachine() {
			state[Hallway.BACK.ordinal()][Side.LEFT.ordinal()] = RT10States.DOORS_STOPPED;
			state[Hallway.FRONT.ordinal()][Side.LEFT.ordinal()] = RT10States.DOORS_STOPPED;
			state[Hallway.BACK.ordinal()][Side.RIGHT.ordinal()] = RT10States.DOORS_STOPPED;
			state[Hallway.FRONT.ordinal()][Side.RIGHT.ordinal()] = RT10States.DOORS_STOPPED;
		}

		public void receive(ReadableDoorMotorPayload msg) {
			updateState(msg);
		}

		private void updateState(ReadableDoorMotorPayload msg) {
			RT10States previousState = state[msg.getHallway().ordinal()][msg.getSide().ordinal()];
			RT10States newState = previousState;
			boolean warningIssued = false;

			if (msg.command() == DoorCommand.NUDGE
					&& !reversal[msg.getHallway().ordinal()][0]
					&& !reversal[msg.getHallway().ordinal()][1]) {

				newState = RT10States.DOORS_NUDGING_NO_REVERSAL;
			} else if (msg.command() == DoorCommand.NUDGE
					&& (reversal[msg.getHallway().ordinal()][0]
							||reversal[msg.getHallway().ordinal()][1])) {
				newState = RT10States.DOOR_NUDGING_AFTER_REVERSAL;
			} else if (msg.command() == DoorCommand.STOP) {
				newState = RT10States.DOORS_STOPPED;
			}

			if (newState != previousState) {
				switch (newState) {
				case DOORS_NUDGING_NO_REVERSAL:
					if (!warningIssued) {
						warning("R-T.10 Violated: at Floor "
								+ currentFloor
								+ " "
								+ msg.getHallway()
								+ " "
								+ msg.getSide()
								+ " Car doors are nudging when no door reversals have occured");
						warningIssued = true;
					}
					break;
				case DOOR_NUDGING_AFTER_REVERSAL:
					
					break;
				case DOORS_STOPPED:
					warningIssued = false;
					break;
				}
			}
			state[msg.getHallway().ordinal()][msg.getSide().ordinal()] = newState;
		}
	}

	private class RT9StateMachine {
		RT9States state;
		RT9States nextState;
		boolean wentToFast;

		public RT9StateMachine() {
			state = RT9States.STOPPED_FAST;
			nextState = RT9States.STOPPED_FAST;
			wentToFast = false;
		}

		public void receive(ReadableDriveSpeedPayload msg) {
			updateState(msg);
		}

		private void updateState(ReadableDriveSpeedPayload msg) {
			nextState = state;

			switch (state) {
			case STOPPED_FAST:
				if (msg.speed() > DriveObject.LevelingSpeed) {
					nextState = RT9States.MOVING;
					wentToFast = false;
				}
				break;
			case STOPPED_NO_FAST:
				if (msg.speed() > DriveObject.LevelingSpeed) {
					nextState = RT9States.MOVING;
					wentToFast = false;
				}
				break;
			case MOVING:
				if (msg.speed() > DriveObject.SlowSpeed) {
					wentToFast = true;
				} else if (msg.speed() <= DriveObject.LevelingSpeed) {
					if (wentToFast) {
						nextState = RT9States.STOPPED_FAST;
					} else {
						nextState = RT9States.STOPPED_NO_FAST;
					}
				}
				break;
			}
			if (nextState != state) {
				switch (nextState) {
				case STOPPED_FAST:

					break;
				case MOVING:

					break;
				case STOPPED_NO_FAST:
					warning("R-T.9 Violated: Car is not reaching fast speed");
					break;
				}
			}
			state = nextState;
		}
	}

	private class RT8_1StateMachine {
		RT8_1States state;
		RT8_1States nextState;

		public RT8_1StateMachine() {
			state = RT8_1States.DOORS_CLOSED;
			nextState = RT8_1States.DOORS_CLOSED;
		}

		public void receive(ReadableDoorOpenPayload msg) {
			updateState(msg);
		}

		private void updateState(ReadableDoorOpenPayload msg) {
			nextState = state;

			switch (state) {
			case DOORS_CLOSED:
				if (msg.isOpen()) {
					for (int i = 0; i < Elevator.numFloors; i++) {
						for (int ii = 0; ii < 4; ii++) {
							if (hallCalls[i][ii] == true) {
								if (upCarLantern || downCarLantern) {
									nextState = RT8_1States.OPEN_ON;
								} else {
									nextState = RT8_1States.OPEN_OFF;
								}
							}
						}
					}
				}
				break;
			case OPEN_ON:
				if (msg.isOpen() == false) {
					nextState = RT8_1States.DOORS_CLOSED;
				}
				break;
			case OPEN_OFF:
				if (msg.isOpen() == false) {
					nextState = RT8_1States.DOORS_CLOSED;
				}
				break;
			}
			if (nextState != state) {
				switch (nextState) {
				case DOORS_CLOSED:
					break;
				case OPEN_ON:
					break;
				case OPEN_OFF:
					warning("R-T.8.1 Violated: Lanterns off while call is pending ");
					break;
				}
			}
			state = nextState;
		}
	}

	private class RT8_2StateMachine {
		RT8_2States state;
		RT8_2States nextState;
		boolean prevUpCarLantern;
		boolean prevDownCarLantern;

		public RT8_2StateMachine() {
			state = RT8_2States.LANTERNS_OFF;
			nextState = RT8_2States.LANTERNS_OFF;
			prevUpCarLantern = false;
			prevDownCarLantern = false;
		}

		public void receive(ReadableCarLanternPayload msg) {
			updateState(msg);
		}

		private void updateState(ReadableCarLanternPayload msg) {
			nextState = state;

			switch (state) {
			case LANTERNS_OFF:
				if ((DoorOpen[0] || DoorOpen[1] || DoorOpen[2] || DoorOpen[3])
						&& (upCarLantern || downCarLantern)) {
					nextState = RT8_2States.LANTERNS_ON;
				}
				break;
			case LANTERNS_ON:
				if (!(DoorOpen[0] || DoorOpen[1] || DoorOpen[2] || DoorOpen[3])) {
					nextState = RT8_2States.LANTERNS_OFF;
				}
				if (prevUpCarLantern != upCarLantern
						|| prevDownCarLantern != downCarLantern) {
					nextState = RT8_2States.LANTERNS_CHANGED;
				}
				break;
			case LANTERNS_CHANGED:
				if (!(DoorOpen[0] || DoorOpen[1] || DoorOpen[2] || DoorOpen[3])) {
					nextState = RT8_2States.LANTERNS_OFF;
				}
				break;
			}
			if (nextState != state) {
				switch (nextState) {
				case LANTERNS_OFF:
					break;
				case LANTERNS_ON:
					break;
				case LANTERNS_CHANGED:
					warning("R-T.8.2 Violated: Lanterns changed while doors open");
					break;
				}
			}
			prevDownCarLantern = downCarLantern;
			prevUpCarLantern = upCarLantern;

			state = nextState;
		}
	}

	private class RT8_3StateMachine {
		RT8_3States state;
		RT8_3States nextState;

		public RT8_3StateMachine() {
			state = RT8_3States.IDLE;
			nextState = RT8_3States.IDLE;
		}

		public void receive(ReadableDriveSpeedPayload msg) {
			updateState(msg);
		}

		private void updateState(ReadableDriveSpeedPayload msg) {
			nextState = state;

			switch (state) {
			case IDLE:
				if(msg.speed()>=DriveObject.FastSpeed && ((msg.direction()==Direction.UP && downCarLantern==true)||(msg.direction()==Direction.DOWN && upCarLantern==true))){
					nextState = RT8_3States.WRONG_DIRECTION;
				}
				break;
			case WRONG_DIRECTION:
				if(msg.speed()<=DriveObject.FastSpeed){
					nextState = RT8_3States.IDLE;
				}
				break;
		}
		
				/*
			
			case MOVING_RIGHT_DIRECTION:
			case MOVING_WRONG_DIRECTION:
				if(msg.speed()==0 && downCarLantern == true){
					nextState = RT8_3States.STOPPED_DOWN;
				}
				else if (msg.speed()==0 && upCarLantern == true){
					nextState = RT8_3States.STOPPED_UP;
				}
				
				break;
			case STOPPED_UP:
				if(!(currentFloor==1)&&msg.speed()>DriveObject.LevelingSpeed && msg.direction()==Direction.DOWN){
					nextState = RT8_3States.MOVING_WRONG_DIRECTION;
				}
				else if (msg.speed()>DriveObject.LevelingSpeed && msg.direction()==Direction.UP){
					nextState = RT8_3States.MOVING_RIGHT_DIRECTION;
				}
				
				else if (msg.speed()==0 && downCarLantern == true){
					nextState = RT8_3States.STOPPED_DOWN;
				}
				
				break;
			case STOPPED_DOWN:
				if(!(currentFloor==8)&&msg.speed()>DriveObject.LevelingSpeed && msg.direction()==Direction.UP){
					nextState = RT8_3States.MOVING_WRONG_DIRECTION;
				}
				else if (msg.speed()>DriveObject.LevelingSpeed && msg.direction()==Direction.DOWN){
					nextState = RT8_3States.MOVING_RIGHT_DIRECTION;
				}
				
				else if (msg.speed()==0 && upCarLantern == true){
					nextState = RT8_3States.STOPPED_UP;
				}
				
				break;

			}
			*/
			if (nextState != state) {
				switch (nextState) {
				//case MOVING_RIGHT_DIRECTION:
					//warning("MOVING_RIGHT_DIRECTION");
					//break;
				//case STOPPED_UP:
					//warning("STOPPED_UP");
					//break;
				//case STOPPED_DOWN:
					//warning("STOPPED_DOWN");
					//break;
				//case MOVING_WRONG_DIRECTION:
					//warning("R-T.8.3 Violated: Lanterns did not match direction of travel");
					//break;
				case IDLE:
					break;
				case WRONG_DIRECTION:
					warning("R-T.8.3 Violated: Lanterns did not match direction of travel");
					break;
				}
			}
			state = nextState;
		}
	}

}
