with Ada.Text_IO;                use Ada.Text_IO;
with Exceptions;                 use Exceptions;
with Real_Type;                  use Real_Type;
--  with Generic_Sliding_Statistics;
--  with Rotations;                  use Rotations;
with Vectors_3D;                 use Vectors_3D;
with Vehicle_Interface;          use Vehicle_Interface;
with Vehicle_Message_Type;       use Vehicle_Message_Type;
--  with Swarm_Structures;           use Swarm_Structures;
with Swarm_Structures_Base;      use Swarm_Structures_Base;

package body Vehicle_Task_Type is

   use Vehicle_Ordered_Map;

   Is_Debug_Print        : constant Boolean  := True;
   Max_Forwards          : constant Integer  := 7;
   New_Coordinator_Delay : constant Positive := 10;
   Release_Delay         : constant Positive := 10;

   procedure Debug_Print (Message : String) is
   begin
      if Is_Debug_Print then
         Put_Line (Message);
      end if;
   end Debug_Print;

   function Calculate_Distance_To_Globe (GlobePos : Positions) return Real is
      VectorDistance : constant Positions := GlobePos - Position;
   begin
      return abs (VectorDistance (x)) + abs (VectorDistance (y)) + abs (VectorDistance (z));
   end;

   function Find_Closest_Globe (Globes : Energy_Globes) return Positions is
      ClosestGlobeDist : Real := Real'Last;
      ClosestGlobePos  : Positions;
      CurrentGlobeDist : Real;
   begin
      for I in Globes'Range loop
         CurrentGlobeDist := Calculate_Distance_To_Globe (Globes (I).Position);
         if CurrentGlobeDist < ClosestGlobeDist then
            ClosestGlobeDist := CurrentGlobeDist;
            ClosestGlobePos  := Globes (I).Position;
         end if;
      end loop;
      return ClosestGlobePos;
   end;

   function Min_Map_Charge (M : Map) return Positive is
      Current_Min_Val : Vehicle_Charges := Vehicle_Charges'Last;
      Current_Min_Key : Positive := M.First_Key;
   begin
      for I in M.Iterate loop
         if Element (I) < Current_Min_Val then
            Current_Min_Val := Element (I);
            Current_Min_Key := Key (I);
         end if;
      end loop;
      return Current_Min_Key;
   end Min_Map_Charge;

   function Max_Map_Charge (M : Map) return Positive is
      Current_Max_Val : Vehicle_Charges := Vehicle_Charges'First;
      Current_Max_Key : Positive := M.First_Key;
   begin
      for I in M.Iterate loop
         if Element (I) > Current_Max_Val then
            Current_Max_Val := Element (I);
            Current_Max_Key := Key (I);
         end if;
      end loop;
      return Current_Max_Key;
   end Max_Map_Charge;

   procedure Combine_Maps (To_Add : in Map; Result : in out Map) is
   begin
      for I in To_Add.Iterate loop
         Result.Include (Key (I), Element (I));
      end loop;
   end Combine_Maps;

   task body Vehicle_Task is

      Vehicle_No         : Positive;
      Current_State      : State := Searching;
      Coordinator_No     : Positive;
      Last_Message       : Inter_Vehicle_Messages;
      Target_Globe_Pos   : Positions;
      Waiting_Vehicles   : Map;
      Wait_Duration      : Integer := 0;
      Lowest_Charge_No   : Positive;
      Greatest_Charge_No : Positive;

      procedure Respond_To_Message_Searching is
      begin
         Receive (Last_Message);
         case Last_Message.Purpose is
            when Broadcast_Globe_Pos =>
               Coordinator_No   := Last_Message.Sender_No;
               Send ((Purpose   => Notify_Of_Charge,
                      Sender_No => Vehicle_No,
                      Target_No => Coordinator_No,
                      Charge    => Current_Charge,
                      Forward_Count => 0
                     ));
               --  Set_Throttle (0.01);
               Target_Globe_Pos := Last_Message.Globe_Pos;
               Current_State    := Waiting_For_Turn;
               Waiting_Vehicles.Clear;
               Debug_Print (Positive'Image (Vehicle_No) &
                              " is now waiting on Coordinator " &
                              Positive'Image (Coordinator_No));

               --  Set_Destination (Target_Globe_Pos);
               --  Set_Throttle (0.2);
               --  Send ((Purpose   => Broadcast_Globe_Pos,
               --         Sender_No => Coordinator_No,
               --         Target_No => Coordinator_No,
               --         Charge    => Last_Message.Charge,
               --         Globe_Pos => Target_Globe_Pos,
               --         Forward_Count => 1
               --        ));
            when others =>
               null;
         end case;
      end Respond_To_Message_Searching;

      procedure Respond_To_Message_Coordinator is
      begin
         Receive (Last_Message);
         case Last_Message.Purpose is
            when Acknowledge =>
               if Last_Message.Target_No = Vehicle_No then
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " Coordinator received release acknowledgment from " &
                                 Positive'Image (Last_Message.Sender_No));
                  if not (Coordinator_No = Last_Message.Sender_No) then
                     Put_Line ("ERROR - Coordinator_No does not match on acknowledge");
                  end if;
               end if;
            when Notify_Of_Charge =>
               if Last_Message.Target_No = Vehicle_No then
                  Send ((Purpose   => Acknowledge,
                         Sender_No => Vehicle_No,
                         Target_No => Last_Message.Sender_No,
                         Charge    => Current_Charge,
                         Forward_Count => 0
                        ));
                  Waiting_Vehicles.Include (Last_Message.Sender_No, Last_Message.Charge);
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " Coordinator got charge info from " &
                                 Positive'Image (Last_Message.Sender_No));
               end if;
            when Broadcast_Globe_Pos =>
               if Last_Message.Forward_Count = 0 then
                  -- update target globe to newer information
                  Target_Globe_Pos := Last_Message.Globe_Pos;
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " Coordinator updated Globe pos");
                  -- multiple competing coordinators
                  if Last_Message.Charge > Current_Charge then
                     -- yield coordinator state to vehicle with greater charge,
                     -- send current waiting vehicles and wait for turn
                     Waiting_Vehicles.Include (Vehicle_No, Current_Charge);
                     Send ((Purpose   => Transfer_Coordinator,
                            Sender_No => Vehicle_No,
                            Target_No => Last_Message.Sender_No,
                            Charge    => Current_Charge,
                            Waiting   => Waiting_Vehicles,
                            Forward_Count => 0
                           ));
                     Coordinator_No := Last_Message.Sender_No;
                     Current_State  := Waiting_For_Turn;
                     Waiting_Vehicles.Clear;
                     Wait_Duration := 0;
                     Debug_Print (Positive'Image (Vehicle_No) &
                                    " yielded Coordinator status to " &
                                    Positive'Image (Coordinator_No));
                  else
                     -- other coordinator must yield
                     Send ((Purpose   => Broadcast_Globe_Pos,
                            Sender_No => Vehicle_No,
                            Target_No => Last_Message.Sender_No,
                            Charge    => Current_Charge,
                            Globe_Pos => Target_Globe_Pos,
                            Forward_Count => 0
                           ));
                  end if;
               end if;
            when Transfer_Coordinator =>
               if Last_Message.Target_No = Vehicle_No then
                  -- accept yielding coordinator and combine waiting vehicles
                  Combine_Maps (Last_Message.Waiting, Waiting_Vehicles);
                  Waiting_Vehicles.Include (Vehicle_No, Current_Charge);
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " accepted Waiting_Vehicles map update from " &
                                 Positive'Image (Last_Message.Sender_No));
               end if;
            when others =>
               null;
         end case;
      end Respond_To_Message_Coordinator;

      procedure Respond_To_Message_Waiting_For_Turn is
      begin
         Receive (Last_Message);
         case Last_Message.Purpose is
            when Acknowledge =>
               if Last_Message.Target_No = Vehicle_No then
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " received charge acknowledgment by Coordinator " &
                                 Positive'Image (Coordinator_No));
                  if not (Coordinator_No = Last_Message.Sender_No) then
                     Put_Line ("ERROR - Coordinator_No does not match on acknowledge");
                  end if;
               end if;
            when Release =>
               if Last_Message.Target_No = Vehicle_No then
                  Send ((Purpose   => Acknowledge,
                         Sender_No => Vehicle_No,
                         Target_No => Last_Message.Sender_No,
                         Charge    => Current_Charge,
                         Forward_Count => 0
                        ));
                  Target_Globe_Pos := Last_Message.Globe_Pos;
                  Set_Destination (Target_Globe_Pos);
                  Current_State := Approaching_Globe;
                  Set_Throttle (1.0);
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " recieved release and travelling to Globe from Coordinator " &
                                 Positive'Image (Coordinator_No));
               elsif Last_Message.Forward_Count < Max_Forwards then
                  Send ((Purpose       => Release,
                         Sender_No     => Last_Message.Sender_No,
                         Target_No     => Last_Message.Target_No,
                         Charge        => Last_Message.Charge,
                         Forward_Count => Last_Message.Forward_Count + 1,
                         Globe_Pos     => Last_Message.Globe_Pos
                        ));
                  --  Debug_Print (Positive'Image (Vehicle_No) &
                  --                 " forwarded release for " &
                  --                 Positive'Image (Last_Message.Target_No));
               end if;
            when Transfer_Coordinator =>
               if Last_Message.Target_No = Vehicle_No then
                  --  Set_Throttle (0.01);
                  -- become new Coordinator
                  Waiting_Vehicles := Last_Message.Waiting;
                  Waiting_Vehicles.Include (Vehicle_No, Current_Charge);
                  Coordinator_No := Vehicle_No;
                  Current_State  := Coordinator;
                  Wait_Duration  := New_Coordinator_Delay;
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " was assigned as new Coordinator from " &
                                 Positive'Image (Last_Message.Sender_No));
               --  elsif Last_Message.Forward_Count < Max_Forwards then
               --     Send ((Purpose       => Transfer_Coordinator,
               --            Sender_No     => Last_Message.Sender_No,
               --            Target_No     => Last_Message.Target_No,
               --            Charge        => Last_Message.Charge,
               --            Forward_Count => Last_Message.Forward_Count + 1,
               --            Waiting       => Last_Message.Waiting
               --           ));
               end if;
            --  when Broadcast_Globe_Pos =>
            --     if Last_Message.Forward_Count < Max_Forwards then
            --        Send ((Purpose       => Broadcast_Globe_Pos,
            --               Sender_No     => Last_Message.Sender_No,
            --               Target_No     => Last_Message.Target_No,
            --               Charge        => Last_Message.Charge,
            --               Forward_Count => Last_Message.Forward_Count + 1,
            --               Globe_Pos     => Last_Message.Globe_Pos
            --              ));
            --     end if;
            when others =>
               null;
         end case;
      end Respond_To_Message_Waiting_For_Turn;

   begin

      -- You need to react to this call and provide your task_id.
      -- You can e.g. employ the assigned vehicle number (Vehicle_No)
      -- in communications with other vehicles.

      accept Identify (Set_Vehicle_No : Positive; Local_Task_Id : out Task_Id) do
         Vehicle_No     := Set_Vehicle_No;
         Local_Task_Id  := Current_Task;
      end Identify;

      -- Replace the rest of this task with your own code.
      -- Maybe synchronizing on an external event clock like "Wait_For_Next_Physics_Update",
      -- yet you can synchronize on e.g. the real-time clock as well.

      -- Without control this vehicle will go for its natural swarming instinct.

      select

         Flight_Termination.Stop;

      then abort

         Outer_task_loop : loop

            Wait_For_Next_Physics_Update;

            if Wait_Duration > 0 then
               Wait_Duration := Wait_Duration - 1;
            end if;

            case Current_State is
            when Searching =>
               -- find nearest globe, if any broadcast and become coordinator
               if Current_Charge < 0.9 and then not (Energy_Globes_Around'Length = 0) then
                  --  Set_Destination (Target_Globe_Pos);
                  --  Set_Throttle (0.2);
                  Target_Globe_Pos := Find_Closest_Globe (Energy_Globes_Around);
                  Send ((Purpose   => Broadcast_Globe_Pos,
                         Sender_No => Vehicle_No,
                         Target_No => Vehicle_No,
                         Charge    => Current_Charge,
                         Globe_Pos => Target_Globe_Pos,
                         Forward_Count => 0
                        ));
                  Waiting_Vehicles.Clear;
                  Waiting_Vehicles.Include (Vehicle_No, Current_Charge);
                  Coordinator_No := Vehicle_No;
                  Current_State  := Coordinator;
                  Wait_Duration  := New_Coordinator_Delay;
                  Debug_Print (Positive'Image (Vehicle_No) & " found Globe, became Coordinator");
               end if;
            when Coordinator =>
               if Wait_Duration < 1 then
                  if Waiting_Vehicles.Is_Empty then
                     Put_Line ("ERROR - no vehicles waiting");
                  else
                     Lowest_Charge_No := Min_Map_Charge (Waiting_Vehicles);
                     if Lowest_Charge_No = Vehicle_No and then Current_Charge < 0.5 then
                        Waiting_Vehicles.Delete (Lowest_Charge_No);
                        if not Waiting_Vehicles.Is_Empty then
                           Greatest_Charge_No := Max_Map_Charge (Waiting_Vehicles);
                           Send ((Purpose   => Transfer_Coordinator,
                                  Sender_No => Vehicle_No,
                                  Target_No => Greatest_Charge_No,
                                  Charge    => Current_Charge,
                                  Waiting   => Waiting_Vehicles,
                                  Forward_Count => 0
                                 ));
                           Debug_Print (Positive'Image (Vehicle_No) &
                                          " transferring Coordinator due to charge " & Vehicle_Charges'Image (Current_Charge) &
                                          " and travelling to Globe, making" & Positive'Image (Greatest_Charge_No) &
                                          " the new Coordinator");
                        else
                           Debug_Print (Positive'Image (Vehicle_No) &
                                          " Coordinator travelling to Globe, Waiting_Vehicles map empty");
                        end if;
                        Set_Destination (Target_Globe_Pos);
                        Set_Throttle (1.0);
                        Current_State := Approaching_Globe;
                        Wait_Duration := 0;
                     elsif not (Lowest_Charge_No = Vehicle_No) then
                        Wait_Duration := Release_Delay;
                        Waiting_Vehicles.Delete (Lowest_Charge_No);
                        Send ((Purpose   => Release,
                               Sender_No => Vehicle_No,
                               Target_No => Lowest_Charge_No,
                               Charge    => Current_Charge,
                               Globe_Pos => Target_Globe_Pos,
                               Forward_Count => 0
                              ));
                        Debug_Print (Positive'Image (Lowest_Charge_No) &
                                       " released from waiting by " &
                                       Positive'Image (Vehicle_No));
                     end if;
                  end if;
               end if;
            when Waiting_For_Turn =>
               null;
               -- forward
            when Approaching_Globe =>
               if Current_Charge > 0.8 then
                  Current_State := Searching;
                  Debug_Print (Positive'Image (Vehicle_No) & " finished travelling to Globe");
                  Set_Throttle (0.5);
                  if not (Energy_Globes_Around'Length = 0) then
                     Send ((Purpose   => Broadcast_Globe_Pos,
                            Sender_No => Coordinator_No,
                            Target_No => Coordinator_No,
                            Charge    => Last_Message.Charge,
                            Globe_Pos => Target_Globe_Pos,
                            Forward_Count => 1
                           ));
                  end if;
                     delay 0.5;
                  -- send ack?
               end if;
            end case;

            while Messages_Waiting loop
               declare
                  Iterations : Integer := 0;
                  Max_Iterations : constant Integer := 100;
               begin
                  case Current_State is
                  when Searching =>
                     Respond_To_Message_Searching;
                  when Coordinator =>
                     Respond_To_Message_Coordinator;
                  when Waiting_For_Turn =>
                     Respond_To_Message_Waiting_For_Turn;
                  when others =>
                     null;
                  end case;
                  if Iterations > Max_Iterations then
                     exit;
                  end if;
                  Iterations := Iterations + 1;
               end;
            end loop;

            --  WaitingQueue.Weight := WaitingQueue.Weight - 1;

            --  if Messages_Waiting then
            --     Receive (LastMessage);
            --     if LastMessage.Purpose = NotifyOfGlobe and then CurrentState = Searching then
            --        TargetGlobePos := LastMessage.GlobePos;
            --        if Current_Charge < LastMessage.ChargeInfo.Charge or else WaitingQueue.Weight < 0 then
            --           WaitingQueue := LastMessage.ChargeInfo;
            --           --  if LastMessage.ChargeInfo.VehicleId = WaitingQueue.VehicleId
            --        end if;
            --        Send ((NotifyOfGlobe, TargetGlobePos, (Vehicle_No, Current_Charge, Weighting))); -- forward
            --        CurrentState := WaitingForTurn;
            --        Set_Throttle (0.1);
            --        Set_Destination (TargetGlobePos);
            --        delay 0.1;
            --     elsif CurrentState = WaitingForTurn then
            --        if Current_Charge < LastMessage.ChargeInfo.Charge or else WaitingQueue.Weight < 0 then
            --           WaitingQueue := LastMessage.ChargeInfo;
            --           --  if LastMessage.ChargeInfo.VehicleId = WaitingQueue.VehicleId
            --        else
            --           Send ((NotifyOfCharge, TargetGlobePos, (Vehicle_No, Current_Charge, Weighting))); -- forward
            --        end if;
            --        if LastMessage.Purpose = NotifyOfGlobe then -- update target globe
            --           TargetGlobePos := LastMessage.GlobePos;
            --           --  Send ((NotifyOfGlobe, TargetGlobePos, (Vehicle_No, Current_Charge, Weighting))); -- forward
            --        end if;
            --     end if;
            --  elsif CurrentState = Searching and then not (Energy_Globes_Around'Length = 0) then
            --     TargetGlobePos := Find_Closest_Globe (Energy_Globes_Around);
            --     Send ((NotifyOfGlobe, TargetGlobePos, (Vehicle_No, Current_Charge, Weighting)));
            --     CurrentState := WaitingForTurn;
            --     Set_Throttle (0.1);
            --     Set_Destination (TargetGlobePos);
            --     delay 0.1;
            --  elsif CurrentState = WaitingForTurn then
            --     if (WaitingQueue.Weight < 1 or else WaitingQueue.Charge < Current_Charge) and then Current_Charge < 0.9 then
            --        Set_Throttle (1.0);
            --        Set_Destination (TargetGlobePos);
            --        CurrentState := ApproachingGlobe;
            --        Put_Line (Positive'Image (Vehicle_No) & " Travelling to Globe");
            --     end if;
            --     Send ((NotifyOfCharge, TargetGlobePos, (Vehicle_No, Current_Charge, Weighting)));
            --  elsif CurrentState = ApproachingGlobe and then Current_Charge > 0.9 then
            --     Set_Throttle (0.5);
            --     CurrentState := Searching;
            --     Put_Line (Positive'Image (Vehicle_No) & " Finished travelling to Globe");
            --     Send ((NotifyOfCharge, TargetGlobePos, (Vehicle_No, Current_Charge, Weighting)));
            --  end if;
         end loop Outer_task_loop;

      end select;

   exception
      when E : others => Show_Exception (E);

   end Vehicle_Task;

end Vehicle_Task_Type;
