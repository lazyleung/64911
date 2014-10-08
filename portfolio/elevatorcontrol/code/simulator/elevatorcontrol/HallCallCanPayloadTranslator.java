package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
import simulator.payloads.translators.CanPayloadTranslator;

public class HallCallCanPayloadTranslator extends BooleanCanTranslator {
	/**
     * CAN translator for messages from atfloor sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
    public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }

    /**
     * CAN translator for messages from atfloor sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
    public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }

}
