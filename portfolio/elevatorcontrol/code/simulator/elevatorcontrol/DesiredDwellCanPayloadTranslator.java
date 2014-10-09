package simulator.elevatorcontrol;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public class DesiredDwellCanPayloadTranslator extends IntegerCanPayloadTranslator {
	public DesiredDwellCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload);
    }

    public DesiredDwellCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload);
    }

	@Override
	public String payloadToString() {
		// TODO Auto-generated method stub
		return "value="+getValue();
	}
}
