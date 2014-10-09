package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class CarCallCanPayloadTranslator extends BooleanCanTranslator {
	
	/**
     * Constructor for WriteableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public CarCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
    	super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public CarCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
    	super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
    }

}
