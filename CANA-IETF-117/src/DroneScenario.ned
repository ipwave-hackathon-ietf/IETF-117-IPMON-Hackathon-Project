//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package cana.src;

//
// TODO auto-generated type
//
network DroneScenario
{
    parameters:
    	@display("i=block/network2");
	    @class(inet::DroneScenario);
	    @signal[traciInitialized](type=bool);
	    double penetrationrate = default(1);
        int dronesnumber = default(1);
        int missionid = default(0);
		string moduleType = default("cana.src.AODVDrone");  // module type to be used in the simulation for each managed vehicle
        string moduleName = default("drone");  // module name to be used in the simulation for each managed vehicle
        string moduleDisplayString = default("i=dronesim/drone");
        double firstStepAt @unit("s") = default(-1s);  // when to start synchronizing with the the setver server (-1: immediately after connecting)
        double updateInterval @unit("s") = default(1s);  // time interval of hosts' position updates        
       	int seed = default(-1); // seed value to set in launch configuration, if missing (-1: current run number)
       	bool autoShutdown = default(true);  // Shutdown module as soon as no more vehicles are in the simulation
       	
}
