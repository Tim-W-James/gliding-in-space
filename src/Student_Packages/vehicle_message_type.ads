-- Suggestions for packages which might be useful:

--  with Ada.Real_Time;         use Ada.Real_Time;
--  with Swarm_Size;            use Swarm_Size;
--  with Vectors_3D;            use Vectors_3D;
with Swarm_Structures_Base; use Swarm_Structures_Base;

package Vehicle_Message_Type is

   -- Replace this record definition by what your vehicles need to communicate.

   type Message_Purpose is (NotifyOfGlobe, NotifyOfCharge);

   type Inter_Vehicle_Messages is record
      Purpose       : Message_Purpose;
      GlobePos      : Positions;
      Charge        : Vehicle_Charges;
   end record;

end Vehicle_Message_Type;
