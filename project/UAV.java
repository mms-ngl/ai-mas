/**
 * A single UAV running over the simulation. 
 * This class implements the class Steppable, the latter requires the implementation 
 * of one crucial method: step(SimState).
 * Please refer to Mason documentation for further details about the step method and how the simulation
 * loop is working.
 * 
 * @author dario albani
 * @mail albani@dis.uniroma1.it
 * @thanks Sean Luke
 */
package sim.app.firecontrol;

import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.Set;
import java.util.Map;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.UUID;  
import java.util.Comparator;

import java.util.Random;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.Double3D;
import sim.util.Int3D;
import sim.util.Int2D;

import java.sql.Timestamp;
import java.util.List; 
import java.util.ArrayList; 
import java.util.Arrays;
import java.util.Collections;

public class UAV implements Steppable{
	private static final long serialVersionUID = 1L;

	// Agent's variable
	public int id; //unique ID
	public double x; //x position in the world
	public double y; //y position in the world
	public double z; //z position in the world
	public Double3D target; //UAV target
	public AgentAction action; //last action executed by the UAV
	public DataPacketType dataType;
	public static double communicationRange = 60; //communication range for the UAVs

	// Agent's local knowledge 
	public Set<WorldCell> knownCells; 
	public Task myTask;
	public String status;
	
	// Agent's settings - static because they have to be the same for all the 
	// UAV in the simulation. If you change it once, you change it for all the UAV.
	public static double linearvelocity = 0.02;
	//0.02;

	//used to count the steps needed to extinguish a fire in a location
	public static int stepToExtinguish = 10;
	//used to remember when first started to extinguish at current location
	private int startedToExtinguishAt = -1;

	public int[] alloc;
	private Map<Task, Integer> tasksPriorities;


	public LinkedList<DataPacket> sentDataPackets;
	public LinkedList<DataPacket> receivedDataPackets;
	public LinkedList<DataPacket> processedDataPackets; 

	public UAV(int id, Double3D myPosition){
		//set agent's id
		this.id = id;
		//set agent's position
		this.x = myPosition.x;
		this.y = myPosition.y;
		this.z = myPosition.z;
		//at the beginning agents have no action
		this.action = null;
		//at the beginning agents have no known cells 
		this.knownCells = new LinkedHashSet<>();

		this.status = null;

		this.sentDataPackets = new LinkedList<>();
		this.receivedDataPackets = new LinkedList<>();
		this.processedDataPackets = new LinkedList<>();


	}

	// DO NOT REMOVE
	// Getters and setters are used to display information in the inspectors
	public int getId(){
		return this.id;
	}

	public void setId(int id){
		this.id = id;
	}

	public double getX(){
		return this.x;
	}

	public void setX(double x){
		this.x = x;
	}

	public double getY(){
		return this.y;
	}

	public void setY(double y){
		this.y = y;
	}

	public double getZ(){
		return this.z;
	}

	public void setZ(double z){
		this.z = z;
	}

	/**
	 *  Do one step.
	 *  Core of the simulation.   
	 */
	public void step(SimState state){
		Ignite ignite = (Ignite)state;

		int anyManager = 0;
		for(Object obj : ignite.UAVs){ 
			UAV other = (UAV) obj;
			if(other.status == "manager"){
				++anyManager;
			}
		}

		if(anyManager == 0){
			this.defineManagers(ignite);
		} 

		AgentAction a = nextAction(ignite);

		//select the next action for the agent
		switch(a){	
		case SELECT_TASK:
			// ------------------------------------------------------------------------
			// this is where your task allocation logic has to go. 
			// be careful, agents have their own knowledge about already explored cells, take that 
			// in consideration if you want to implement an efficient strategy.
			// TODO Implement here your task allocation strategy
			
			selectTask(ignite); // <- change the signature if needed

			this.action = a; //why in case? 
			break;

		case SELECT_CELL:
			// ------------------------------------------------------------------------
			// this is where your random walk or intra-task allocation logic has to go. 
			// be careful, agents have their own knowledge about already explored cells, take that 
			// in consideration if you want to implement an efficient strategy.
			// TODO Implement here your random walk or intra-task allocation strategy
			//System.err.println("TODO: and now? Use random walk or task assignment!");

			selectCell(ignite); //<- change the signature if needed
			
		case MOVE:
			move(state);
			break;

		case EXTINGUISH:
			//if true set the cell to be normal and foamed
			if(extinguish(ignite)){
				//retrieve discrete location of this
				Int3D dLoc = ignite.air.discretize(new Double3D(this.x, this.y, this.z));
				//extinguish the fire
				((WorldCell)ignite.forest.field[dLoc.x][dLoc.y]).extinguish(ignite);
				this.target=null;
			}

			this.action = a;
			break;

		default:	
			break;
		}
	}

	/**
	 * What to do next?
	 * TODO Feel free to modify this at your own will in case you have a better 
	 * strategy
	 */ 
	private AgentAction nextAction(Ignite ignite){
		//if I do not have a task I need to take one
		if(this.myTask == null){
			return AgentAction.SELECT_TASK;
		}
		//else, if I have a task but I do not have target I need to take one
		else if(this.target == null){
			return AgentAction.SELECT_CELL;
		}
		//else, if I have a target and task I need to move toward the target
		//check if I am over the target and in that case execute the right action;
		//if not, continue to move toward the target
		else if(this.target.equals(ignite.air.discretize(new Double3D(x, y, z)))){
			//if on fire then extinguish, otherwise move on
			WorldCell cell = (WorldCell)ignite.forest.field[(int) x][(int) y];

			//store the knowledge for efficient selection
			this.knownCells.add(cell);

			//TODO maybe, you can share the knowledge about the just extinguished cell here!
			if(cell.type.equals(CellType.FIRE))
				return AgentAction.EXTINGUISH;
			else
				return AgentAction.SELECT_CELL;

		}else{
			return AgentAction.MOVE;
		}		
	}

	/**
	 * Take the centroid of the fire and its expected radius and extract the new
	 * task for the agent.
	 */
	private void selectTask(Ignite ignite) {
		
		if(this.status == "manager" || this.status == "_manager"){
		
			int numBid = 0;
			int numPropose = 0;
			Map<DataPacket, DataPacket> proposesForBids = new HashMap<>();
			for(DataPacket sp : this.sentDataPackets){
				if(sp.payload.type.equals(DataPacketType.BID)){
					++numBid;
					for(DataPacket rp : this.receivedDataPackets){
						if(rp.payload.type.equals(DataPacketType.PROPOSE)){
							if(sp.header.packetID == rp.payload.resPacketID){
								proposesForBids.put(sp, rp);
								++numPropose;
								break;
							}

						}
					}
				}
			}

			if(((this.status == "manager") && (numBid == numPropose) && numPropose > 0) || (this.status == "_manager")){

				Map<Task, Map<Integer, Double>> proposedUtils = new HashMap<Task, Map<Integer, Double>>();
				for(Task task : ignite.tasks){
					Map<Integer, Double> idUtils = new HashMap<Integer, Double>();
					for(Map.Entry<DataPacket, DataPacket> entry : proposesForBids.entrySet()){
						Task tsk = (Task) entry.getKey().payload.object;
						if(tsk == task){
							int id = entry.getValue().header.senderID;
							double util = (double) entry.getValue().payload.object;

							idUtils.put(id, util);
						}
					}
					double u = this.utilFunction(task, ignite);
					idUtils.put(this.id, u);
					proposedUtils.put(task, idUtils);
				}

				int neighbors = 1;
				for(Object _obj : ignite.UAVs){ 
					UAV neighbor = (UAV) _obj;
					if(neighbor != this){
						if(isInCommunicationRange(new Double3D(neighbor.x, neighbor.y, neighbor.z))){
							++neighbors;
						}						
					}
				}

				this.alloc = new int[ignite.tasks.size()];
				this.tasksPriorities = this.defineTaskPriorities(ignite, neighbors);


				for(int i = 0; i < neighbors; ++i){
					double minUtil = 1.0;
					Task task = null;
					int id = -1;

					for(Map.Entry<Task, Map<Integer, Double>> entry : proposedUtils.entrySet()){

						Task tsk = entry.getKey();
						
						if(this.alloc[ignite.tasks.indexOf(tsk)] < this.tasksPriorities.get(tsk)){

							Map<Integer, Double> temp = new HashMap<Integer, Double>(entry.getValue());

							Map<Integer, Double> _temp = new LinkedHashMap<>();
							temp.entrySet().stream().sorted(Map.Entry.comparingByValue())
				                                .forEachOrdered(x -> _temp.put(x.getKey(), x.getValue()));

							if(!_temp.isEmpty()){

								Map.Entry<Integer, Double> item = _temp.entrySet().iterator().next();
								int key = item.getKey();
								double value = item.getValue(); 

								if(minUtil > value){
									minUtil = value;
									task = entry.getKey();
									id = key;
								}
							}
						}
					}

					if(task != null){
						for(Map.Entry<Task, Map<Integer, Double>> entry : proposedUtils.entrySet()){
							
							Map<Integer, Double> temp = entry.getValue();
							final int _id = id;
							temp.entrySet().removeIf(_entry -> _entry.getKey().equals(_id)); 	
						}

						this.alloc[ignite.tasks.indexOf(task)]++;
					}

					if(id == this.id){
						this.myTask = task;	
						this.target = new Double3D(this.myTask.centroid.x, this.myTask.centroid.y, z);	} else {
						UUID uniqueID = UUID.randomUUID();
						Timestamp timestamp = new Timestamp(System.currentTimeMillis());
						DataPacket packet = new DataPacket(uniqueID, this.id, timestamp, 
							                               DataPacketType.AWARD, null, task);

						this.sendDataOne(ignite, id, packet);
					}
				}
			}

			
			if(this.sentDataPackets.size() == 0 && this.status == "manager"){
				for(Task task : ignite.tasks){ 
					this.sendDataMany(ignite, task);
				}
			}

			int counter = 0;
			for(Object obj : ignite.UAVs){ 
				UAV uav = (UAV) obj;
				if(uav.status == "manager"){
					++counter;
				}
			}

		} else if(this.status == null){
			for(DataPacket p : this.receivedDataPackets){
				
				if(this.receiveData(p) && (p.payload.type.equals(DataPacketType.BID))){
					Task task = (Task) p.payload.object;
					double util = this.utilFunction(task, ignite);

					UUID uniqueID = UUID.randomUUID();
					Timestamp timestamp = new Timestamp(System.currentTimeMillis());
					DataPacket packet = new DataPacket(uniqueID, this.id, timestamp, 
						                               DataPacketType.PROPOSE, p.header.packetID, util);

					this.sendDataOne(ignite, p.header.senderID, packet);

					this.processedDataPackets.add(p);
				} else if(this.receiveData(p) && (p.payload.type.equals(DataPacketType.AWARD))){

					Task task = (Task) p.payload.object;
					DataPacketType dataType = DataPacketType.ACCEPT;
					
					if(this.myTask == null){
						this.myTask = task;
						this.target = new Double3D(this.myTask.centroid.x, this.myTask.centroid.y, z);
					} else {
						dataType = DataPacketType.REFUSE;
					}

					UUID uniqueID = UUID.randomUUID();
					Timestamp timestamp = new Timestamp(System.currentTimeMillis());
					DataPacket packet = new DataPacket(uniqueID, this.id, timestamp, 
						                               dataType, p.header.packetID, task);
					
					this.sendDataOne(ignite, p.header.senderID, packet);
					
					this.processedDataPackets.add(p);
				}
			}
		} 
	}

	private double utilFunction(Task t, Ignite ignite){
		double numMoves = Math.max(Math.abs(t.centroid.x - this.x), Math.abs(t.centroid.y - this.y));
		int maxMoves = Math.max(ignite.width, ignite.height);
		double utilMoves = 0.7 * ((double) numMoves / maxMoves);

		int taskSize = t.cells.size();
		int maxTaskSize = ignite.width * ignite.height;
		double utilTaskSize = 0.2 * (1.0 - (double) taskSize / maxTaskSize);

		double taskRadius = t.radius;
		double maxTaskRadius = Math.sqrt(ignite.width * ignite.width + ignite.height * ignite.height);
		double utilTaskRadius = 0.1 * (1.0 - (double) taskRadius / maxTaskRadius);

		double util = utilMoves + utilTaskSize + utilTaskRadius;

		return util;
	}

	/**
	 * Take the centroid of the fire and its expected radius and select the next 
	 * cell that requires closer inspection or/and foam. 
	 */
	private void selectCell(Ignite ignite) {
		//remember to set the new target at the end of the procedure
		Double3D newTarget = null;

		int taskRadius = (int) Math.ceil(this.myTask.radius);

		Int2D centroid = this.myTask.centroid;

		WorldCell lastCell = (WorldCell) this.knownCells.toArray()[ this.knownCells.size()-1 ];

		int radius = 1;

		int minI = -radius;
		int minJ = -radius;

		int maxI = radius;
		int maxJ = radius;

		int dx = lastCell.x - centroid.x;
		int dy = lastCell.y - centroid.y;

		if(lastCell.type.equals(CellType.BURNED)){
			// Int2D pos = new Int2D(lastCell.x, lastCell.y); 
			// radius = (int) pos.distance(centroid);
			if( dx > 0 && dy < 0){
				minI = 0;
			} else if( dx < 0 && dy < 0){
				maxJ = 0;
			} else if( dx < 0 && dy > 0){
				maxI = 0;
			} else if( dx > 0 && dy > 0){
				minJ = 0;
			}
		} 


		List<Int2D> list = new ArrayList<>();
		//int radius = (int) Math.ceil(this.myTask.radius);

		while(radius <= taskRadius){
			for(int i=minI; i<=maxI; i++){
				for(int j=minJ; j<=maxJ; j++){

					int x = lastCell.x + i;
					int y = lastCell.y + j;
					if(x >= 0 && x < ignite.width && y >= 0 && y < ignite.height){

						WorldCell cell = (WorldCell)ignite.forest.field[(int) x][(int) y];
						Int2D pos = new Int2D(x, y); 
						double r = pos.distance(centroid);
						if(!this.knownCells.contains(cell) && r <= taskRadius){
							list.add(pos);
						}
					} 
				}
			}

			if(list.isEmpty()) {
				radius++;

				maxI += 1;
				maxJ += 1;
				minI -= 1;
				minJ -= 1;
			} else {
				break;
			}
		}

		if(list.isEmpty()){

			this.myTask = null;

			newTarget = new Double3D(lastCell.x, lastCell.y, z);

		} else {

		    Random rand = new Random();
		    Int2D allowedPos = list.get(rand.nextInt(list.size()));
			newTarget = new Double3D(allowedPos.x, allowedPos.y, z);
		}

		this.target = newTarget;
	}


	private int randomIndent(){
     	Random rand = new Random();
    	return rand.nextInt((1 - (-1)) + 1) + (-1);
	}

	/**
	 * Move the agent toward the target position
	 * The agent moves at a fixed given velocity
	 * @see this.linearvelocity
	 */
	public void move(SimState state){
		Ignite ignite = (Ignite) state;

		// retrieve the location of this 
		Double3D location = ignite.air.getObjectLocationAsDouble3D(this);
		double myx = location.x;
		double myy = location.y;
		double myz = location.z;

		// compute the distance w.r.t. the target
		// the z axis is only used when entering or leaving an area
		double xdistance = this.target.x - myx;
		double ydistance = this.target.y - myy;

		if(xdistance < 0)
			myx -= Math.min(Math.abs(xdistance), linearvelocity);
		else
			myx += Math.min(xdistance, linearvelocity);

		if(ydistance < 0){ 
			myy -= Math.min(Math.abs(ydistance), linearvelocity); 
		}
		else{	
			myy += Math.min(ydistance, linearvelocity); 
		}

		// update position in the simulation
		ignite.air.setObjectLocation(this, new Double3D(myx, myy, myz));
		// update position local position
		this.x = myx;
		this.y = myy;
		this.z = myz;
	}

	/**
	 * Start to extinguish the fire at current location.
	 * @return true if enough time has passed and the fire is gone, false otherwise
	 * @see this.stepToExtinguish
	 * @see this.startedToExtinguishAt
	 */
	private boolean extinguish(Ignite ignite){
		if(startedToExtinguishAt==-1){
			this.startedToExtinguishAt = (int) ignite.schedule.getSteps();
		}
		//enough time has passed, the fire is gone
		if(ignite.schedule.getSteps() - startedToExtinguishAt == stepToExtinguish){
			startedToExtinguishAt = -1;
			return true;
		}		
		return false;
	}

	/**
	 * COMMUNICATION
	 * Check if the input location is within communication range
	 */
	public boolean isInCommunicationRange(Double3D otherLoc){
		Double3D myLoc = new Double3D(x,y,z);
		return myLoc.distance(otherLoc) <= UAV.communicationRange;
	}

	/**
	 * COMMUNICATION
	 * Send a message to the team
	 */
	public void sendDataMany(Ignite ignite, Task task){
		for(Object obj : ignite.UAVs){ //count also this uav
			UAV other = (UAV) obj;
			if(other != this) {
				if(isInCommunicationRange(new Double3D(other.x, other.y, other.z))){
					UUID uniqueID = UUID.randomUUID();
					Timestamp timestamp = new Timestamp(System.currentTimeMillis());  
					DataPacket packet = new DataPacket(uniqueID, this.id, timestamp, 
						                               DataPacketType.BID, null, task);

					other.receivedDataPackets.add(packet);
					this.sentDataPackets.add(packet);
				}
			}
		}
	}

	public void sendDataOne(Ignite ignite, int id, DataPacket packet){
		for(Object obj : ignite.UAVs){ //count also this uav
			UAV other = (UAV) obj;
			if(other.id == id) {
				other.receivedDataPackets.add(packet);
				this.sentDataPackets.add(packet);
			}
		}
	}


	/**
	 * COMMUNICATION
	 * Receive a message from the team
	 */
	public boolean receiveData(DataPacket packet){
		return !(this.processedDataPackets.contains(packet) ? true : false);
	}

	/**
	 * COMMUNICATION
	 * Retrieve the status of all the agents in the communication range.
	 * @return an array of size Ignite.tasks.size()+1 where at position i you have 
	 * the number of agents enrolled in task i (i.e. Ignite.tasks.get(i)). 
	 * 
	 * HINT: you can easily assume that the number of uncommitted agents is equal to:
	 * Ignite.numUAVs - sum of all i in the returned array
	 */
	public int[] retrieveAgents(Ignite ignite){
		int[] status = new int[ignite.tasks.size()];
		
		for(Object obj : ignite.UAVs){ //count also this uav
			UAV other = (UAV) obj;
			
			Task task = other.myTask;
			if(task != null)
				status[ignite.tasks.indexOf(task)]++;
		}
		
		return status;
	}
	
	@Override
	public boolean equals(Object obj){
		UAV uav = (UAV) obj;
		return uav.id == this.id;
	}
	
	@Override
	public String toString(){ 
		return id+"UAV-"+x+","+y+","+z+"-"+action;
	} 	

	private Map<Task, Integer> defineTaskPriorities(Ignite ignite, int numUAVs){
		Map<Task, Double> utilTasks = new HashMap<>();
		for(Task t : ignite.tasks){
			int taskSize = t.cells.size();
			int maxTaskSize = ignite.width * ignite.height;
			double utilTaskSize = 0.7 * ((double) taskSize / maxTaskSize);

			double taskRadius = t.radius;
			double maxTaskRadius = Math.sqrt(ignite.width * ignite.width + ignite.height * ignite.height);
			double utilTaskRadius = 0.3 * ((double) taskRadius / maxTaskRadius);
			
			double util = utilTaskSize + utilTaskRadius;
			utilTasks.put(t, util);
		}

		//new map sorted by priority values	
		Map<Task, Double> _utilTasks = new LinkedHashMap<>();
		utilTasks.entrySet().stream()
		    .sorted(Map.Entry.comparingByValue(Comparator.reverseOrder()))
		    .forEachOrdered(x -> _utilTasks.put(x.getKey(), x.getValue()));


		int w = numUAVs / ignite.tasks.size();
		int r = numUAVs % ignite.tasks.size();

		Map<Task, Integer> tasksPriorities = new HashMap<>();
		for(Task t : ignite.tasks){
			tasksPriorities.put(t, w);
		}
		
		int counter = 0;
		for(Map.Entry<Task, Double> entry : _utilTasks.entrySet()){
			if(counter < r){
				tasksPriorities.put(entry.getKey(), tasksPriorities.get(entry.getKey()) + 1);			
			}
			++counter;
		}

		return tasksPriorities;
	}

	public void defineManagers(Ignite ignite){
		Map<UAV, LinkedList<UAV>> items = new HashMap<>();
		for(Object obj : ignite.UAVs){ 

			UAV current = (UAV) obj;

			LinkedList<UAV> neighbors = new LinkedList<>();
			for(Object _obj : ignite.UAVs){ 
				UAV neighbor = (UAV) _obj;
				if(current != neighbor){
					Double3D currentLoc = new Double3D(current.x, current.y, current.z);
					Double3D neighborLoc = new Double3D(neighbor.x, neighbor.y, neighbor.z);

					if(currentLoc.distance(neighborLoc) <= UAV.communicationRange){
						neighbors.add(neighbor);
					}
				}
			}

			items.put(current, neighbors);
		}

		while(!items.isEmpty()){
			int max = 0;
			UAV manager = null;

			for(Map.Entry<UAV, LinkedList<UAV>> entry : items.entrySet()){
				int size = entry.getValue().size();
				if( size >= max){
					manager = entry.getKey();
					max = size;
				}
			}

			if(items.get(manager).isEmpty()){
				manager.status = "_manager";
			} else {
				manager.status = "manager";
			}

			for(Object _obj : items.get(manager)){
				items.remove(_obj);
			}
			items.remove(manager);
		}
	}
}


