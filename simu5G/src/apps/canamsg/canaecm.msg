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

//
// TODO generated message class
//

import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.common.geometry.Geometry;

cplusplus {{
const inet::B D2D_MULTIHOP_HEADER_LENGTH = inet::B(8);
}}

class Canaecm  extends inet::FieldsChunk{
	unsigned int sno;
    simtime_t payloadTimestamp;
  	unsigned int srcId;
    uint32_t msgid;          // unique id of the message  < sender id  | local msg id >    
    inet::Coord AccidCoord;
    double maxRadius;  
    int ttl;                 // time-to-live (to avoid flooding)
    unsigned int hops;       // number of hops
}
