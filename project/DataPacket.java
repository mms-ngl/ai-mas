/*
 * Simple structure for a data packet.
 * 
 * @author dario albani
 * @mail dario.albani@istc.cnr.it
 */

package sim.app.firecontrol;
import java.util.LinkedList;
import java.sql.Timestamp;
import java.util.UUID;  

public class DataPacket{

	public class Header{
		public UUID packetID;
		public int senderID;
		public Timestamp timestamp; 

		public Header(UUID packetID, int senderID, Timestamp timestamp){
			this.packetID = packetID;
			this.senderID = senderID;
			this.timestamp = timestamp;
		}
	};

	public class Payload{
		public DataPacketType type;
		public UUID resPacketID;
		public Object object;

		public Payload(DataPacketType type, UUID resPacketID, Object object){
			this.type = type;
			this.resPacketID = resPacketID;
			this.object = object;
		}
	};

	public Header header;
	public Payload payload;

	public DataPacket(UUID packetID, int senderID, Timestamp timestamp, DataPacketType type, UUID resPacketID, Object object){
		this.header = new Header(packetID, senderID, timestamp);
		this.payload = new Payload(type, resPacketID, object);
	}
}
