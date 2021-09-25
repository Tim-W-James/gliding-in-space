--  with Ada.Real_Time;         use Ada.Real_Time;
--  with Swarm_Size;            use Swarm_Size;
--  with Vectors_3D;            use Vectors_3D;
with Swarm_Structures_Base; use Swarm_Structures_Base;
with Ada.Containers.Indefinite_Ordered_Maps;

package Vehicle_Message_Type is

   package Vehicle_Ordered_Map is new
     Ada.Containers.Indefinite_Ordered_Maps
       (Key_Type        => Positive,
        Element_Type    => Vehicle_Charges);
   use Vehicle_Ordered_Map;

   type Message_Purpose is (Broadcast_Globe_Pos, Notify_Of_Charge, Acknowledge, Transfer_Coordinator, Release);
   type Inter_Vehicle_Messages (Purpose : Message_Purpose := Acknowledge) is record
      Sender_No      : Positive;
      Target_No      : Positive;
      Charge         : Vehicle_Charges;
      Forward_Count  : Integer := 0;
      case Purpose is
            when Broadcast_Globe_Pos | Release =>
               Globe_Pos : Positions;
            when Transfer_Coordinator =>
               Waiting   : Map;
            when others =>
               null;
      end case;
   end record;

end Vehicle_Message_Type;
