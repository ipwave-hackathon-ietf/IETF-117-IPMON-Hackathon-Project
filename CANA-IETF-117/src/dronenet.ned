
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
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.AdhocHost;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.networklayer.ipv4.RoutingTableRecorder;
//simple Drone
//{
//    parameters:
//        double posX = 0;
//        double posY = 0;
//        double destination_x = 0;
//        double destination_y = 0;
//
//
//    gates:
//        input in @directIn;
//}

network dronenet{
    parameters:
        int numHosts = default(2);
        int x_size;
        int y_size;
        int numdest = default(1);
        @display("bgb=405,326");

    submodules:

        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=91,31");
        }
        routingTableRecorder: RoutingTableRecorder {
            parameters:
                @display("p=91,148");
        }

        gcs: AdhocHost {
            @display("i=device/antennatower_vl;r=,,#707070;p=227,148");
        }

//        drone[numHosts]: Drone;
}