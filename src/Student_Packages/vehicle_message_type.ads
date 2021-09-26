with Swarm_Structures_Base; use Swarm_Structures_Base;

package Vehicle_Message_Type is

   type Message_Purpose is (Broadcast_Globe_Pos, Notify_Of_Charge, Transfer_Coordinator, Release);
   type Inter_Vehicle_Messages (Purpose : Message_Purpose := Notify_Of_Charge) is record
      Sender_No      : Positive;
      Target_No      : Positive;
      Charge         : Vehicle_Charges;
      Forward_Count  : Integer := 0;
      case Purpose is
            when Broadcast_Globe_Pos | Release =>
               Globe_Pos : Positions;
            when others =>
               null;
      end case;
   end record;

end Vehicle_Message_Type;
