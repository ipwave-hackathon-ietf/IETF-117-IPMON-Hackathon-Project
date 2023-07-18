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
import inet.mobility.base.MovingMobilityBase;

//
// TODO auto-generated type
//
simple DroneNetMob extends MovingMobilityBase
{
    parameters:
        int ndst = default (2);
        int nd = default (0);
        double weightCapacity = default(20);
        double remainingBattery = default(100);
        int parcelSelecitionMethod = default(4);
        double initialX @unit(m) = default(uniform(constraintAreaMinX, constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(constraintAreaMinY, constraintAreaMaxY));
        double initialZ @unit(m) = default(nanToZero(uniform(constraintAreaMinZ, constraintAreaMaxZ)));
        double initialMovementHeading @unit(deg) = default(uniform(0deg, 360deg)); // initial heading
        double initialMovementElevation @unit(deg) = default(0deg); // initial elevation
        bool initFromDisplayString = default(true); // enables one time initialization from the subject module's display string
        volatile double changeInterval @unit(s); // frequency of changing speed and angle (can be random)
        volatile double angleDelta @unit(deg); // rotate velocity vector by this much around the rotation axis defined by the rotationAxisAngle parameter (can be random)
        volatile double rotationAxisAngle @unit(deg) = default(0deg); // defines a vector in the plane perpendicular to the velocity, where 0 deg means the Z axis when bank is zero (can be random)
        volatile double speed @unit(mps); // speed (can be random, updated every changeInterval)
        @class(inet::DroneNetMob);    
}
