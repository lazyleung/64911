//18649 Fall 2014 group 11 Eric Newhall enewhall Jonathan Leung kjleung Ting Xu tingx Mengzhe Li mzli 


package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;
import simulator.elevatorcontrol.*;
import simulator.framework.*;
import simulator.elevatormodules.*;

public class DoorMotorCanPayloadTranslator extends CanPayloadTranslator {

    /**
     * Constructor for WriteableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DoorMotorCanPayloadTranslator(WriteableCanMailbox payload, Hallway hallway, Side side) {
        super(payload, 4, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DoorMotorCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway, Side side) {
        super(payload, 4, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
    }
    
    
    public void set(DoorCommand dc) {
        setValue(dc);
    }

    public void setValue(DoorCommand dc) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, dc.ordinal(), 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public DoorCommand getValue() {
	int val = getIntFromBitset(getMessagePayload(), 0,32);
	for(DoorCommand dc : DoorCommand.values()){
	    if(dc.ordinal() == val){
		return dc;
	    }
	}
	return(null);
    }

    public String payloadToString() {
        return "DoorCommand = " + getValue();
    }
}
