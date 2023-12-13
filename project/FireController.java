package sim.app.firecontrol;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import sim.engine.SimState;
import sim.engine.Steppable;

public class FireController implements Steppable{
	private static final long serialVersionUID = 1L;
	
	@Override
	public void step(SimState state) {

		if(Ignite.cellsOnFire == 0){
			String fileName = System.getProperty("user.dir") + "/" + System.currentTimeMillis() + ".txt";
			
			try {
				FileWriter fw = new FileWriter(new File(fileName),true);
				BufferedWriter bwr = new BufferedWriter(fw);
				bwr.append("Ignite.cellsOnFire: " + Ignite.cellsOnFire);
				bwr.append("Ignite.cellsBurned: " + Ignite.cellsBurned);
				bwr.flush();
				bwr.close();
			} catch (IOException e) {
				System.err.println("Exception in FireControll.step() " + e.toString());
				e.printStackTrace();
			}
			
			state.kill();
		}
	}

}
