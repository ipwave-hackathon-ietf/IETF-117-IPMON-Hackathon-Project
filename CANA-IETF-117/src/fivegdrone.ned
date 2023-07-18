//
// Copyright (C) 2014 OpenSim Ltd.
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

import simu5g.nodes.Ue;
import inet.applications.contract.IApp;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.contract.IRoutingTable;
import inet.networklayer.contract.INetworkLayer;
import inet.networklayer.configurator.ipv4.HostAutoConfigurator;
import inet.transportlayer.tcp.Tcp;
import inet.transportlayer.udp.Udp;
import inet.common.MessageDispatcher;
import simu5g.nodes.Ue;
import simu5g.nodes.NR.NRUe;
import cana.src.DroneNetMob;

//
//Drone Module With 5G protocol stack
//
module fivegdrone extends Ue
{
    parameters:
        
        @display("i=device/car");

        //# Mobility
        mobilityType = default("DroneNetMob");

        //# Network Layer specs
        *.routingTableModule = default(absPath(".ipv4.routingTable"));
        
        ipv4.configurator.typename = "HostAutoConfigurator";
        ipv4.configurator.interfaces = "wlan";
        
        double weightCapacity = default(10);
        double batteryCapacity = default(20);
    	int parcelSelecitionMethod = default(2); 
    	
    	submodules:
	    configurator: HostAutoConfigurator {
            @display("p=126.4,413.96");
        }	
}
