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
   Max_Forwards          : constant Integer  := 5;
   Coordinator_Dist      : constant Real     := 0.01;
   Waiting_Dist          : constant Real     := 0.25;
   Release_Delay         : constant Positive := 5;
   Notify_Delay          : constant Positive := 20;

   procedure Debug_Print (Message : String) is
   begin
      if Is_Debug_Print then
         Put_Line (Message);
      end if;
   end Debug_Print;

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
      Pause_Duration     : Integer := 0;
      Lowest_Charge_No   : Positive;

      function Calculate_Distance_To_Pos (Pos : Positions) return Real is
         VectorDistance : constant Positions := Pos - Position;
      begin
         return abs (VectorDistance (x)) + abs (VectorDistance (y)) + abs (VectorDistance (z));
      end;

      function Find_Closest_Globe (Globes : Energy_Globes) return Positions is
         ClosestGlobeDist : Real := Real'Last;
         ClosestGlobePos  : Positions;
         CurrentGlobeDist : Real;
      begin
         for I in Globes'Range loop
            CurrentGlobeDist := Calculate_Distance_To_Pos (Globes (I).Position);
            if CurrentGlobeDist < ClosestGlobeDist then
               ClosestGlobeDist := CurrentGlobeDist;
               ClosestGlobePos  := Globes (I).Position;
            end if;
         end loop;
         return ClosestGlobePos;
      end;

      procedure Respond_To_Message_Searching is
      begin
         Receive (Last_Message);
         case Last_Message.Purpose is
            when Broadcast_Globe_Pos =>
               -- Globe located via Coordinator, wait for turn
               Send ((Purpose   => Notify_Of_Charge,
                      Sender_No => Vehicle_No,
                      Target_No => Last_Message.Sender_No,
                      Charge    => Current_Charge,
                      Forward_Count => 0
                     ));
               Coordinator_No   := Last_Message.Sender_No;
               Target_Globe_Pos := Last_Message.Globe_Pos;
               Current_State    := Waiting_For_Turn;
               Waiting_Vehicles.Clear;
               Pause_Duration := Notify_Delay;
               --  Debug_Print (Positive'Image (Vehicle_No) &
               --                 " is now waiting on Coordinator " &
               --                 Positive'Image (Coordinator_No));
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
                  Waiting_Vehicles.Exclude (Last_Message.Sender_No);
               end if;
            when Notify_Of_Charge =>
               if Last_Message.Target_No = Vehicle_No then
                  Waiting_Vehicles.Include (Last_Message.Sender_No, Last_Message.Charge);
                  Debug_Print (Positive'Image (Vehicle_No) &
                                 " Coordinator got charge info from " &
                                 Positive'Image (Last_Message.Sender_No));
               end if;
            when Transfer_Coordinator =>
               null;
            when others =>
               null;
         end case;
      end Respond_To_Message_Coordinator;

      procedure Respond_To_Message_Waiting_For_Turn is
      begin
         Receive (Last_Message);
         case Last_Message.Purpose is
            when Broadcast_Globe_Pos =>
               Target_Globe_Pos := Last_Message.Globe_Pos;
               if Last_Message.Forward_Count < Max_Forwards then
                  -- forward Globe position to reach additional vehicles
                  Send ((Purpose   => Broadcast_Globe_Pos,
                         Sender_No => Last_Message.Sender_No,
                         Target_No => Last_Message.Sender_No,
                         Charge    => Current_Charge,
                         Globe_Pos => Target_Globe_Pos,
                         Forward_Count => Last_Message.Forward_Count + 1
                        ));
                  --  Debug_Print (Positive'Image (Vehicle_No) &
                  --                 " forwarded Globe pos from " &
                  --                 Positive'Image (Last_Message.Sender_No));
               end if;
            when Release =>
               if Last_Message.Target_No = Vehicle_No then
                  Target_Globe_Pos := Last_Message.Globe_Pos;
                  Current_State := Approaching_Globe;
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
               null;
            when others =>
               null;
         end case;
      end Respond_To_Message_Waiting_For_Turn;

      procedure Respond_To_Message_Approaching_Globe is
      begin
         Receive (Last_Message);
         case Last_Message.Purpose is
            when Broadcast_Globe_Pos =>
               Target_Globe_Pos := Last_Message.Globe_Pos;
               if Last_Message.Forward_Count < Max_Forwards then
                  -- forward Globe position to reach additional vehicles
                  Send ((Purpose   => Broadcast_Globe_Pos,
                         Sender_No => Last_Message.Sender_No,
                         Target_No => Last_Message.Sender_No,
                         Charge    => Current_Charge,
                         Globe_Pos => Target_Globe_Pos,
                         Forward_Count => Last_Message.Forward_Count + 1
                        ));
                  --  Debug_Print (Positive'Image (Vehicle_No) &
                  --                 " forwarded Globe pos from " &
                  --                 Positive'Image (Last_Message.Sender_No));
               end if;
            when Release =>
               null;
               --  if Last_Message.Forward_Count < Max_Forwards then
               --     Send ((Purpose       => Release,
               --            Sender_No     => Last_Message.Sender_No,
               --            Target_No     => Last_Message.Target_No,
               --            Charge        => Last_Message.Charge,
               --            Forward_Count => Last_Message.Forward_Count + 1,
               --            Globe_Pos     => Last_Message.Globe_Pos
               --           ));
               --     Debug_Print (Positive'Image (Vehicle_No) &
               --                    " forwarded release for " &
               --                    Positive'Image (Last_Message.Target_No));
               --  end if;
            when Transfer_Coordinator =>
               null;
            when others =>
               null;
         end case;
      end Respond_To_Message_Approaching_Globe;

   begin

      -- You need to react to this call and provide your task_id.
      -- You can e.g. employ the assigned vehicle number (Vehicle_No)
      -- in communications with other vehicles.

      accept Identify (Set_Vehicle_No : Positive; Local_Task_Id : out Task_Id) do
         Vehicle_No     := Set_Vehicle_No;
         Local_Task_Id  := Current_Task;
      end Identify;

      -- Without control this vehicle will go for its natural swarming instinct.

      select

         Flight_Termination.Stop;

      then abort

         Outer_task_loop : loop

            Wait_For_Next_Physics_Update;

            -- message receiving
            while Messages_Waiting loop
               --  declare
               --     --  Iterations : Integer := 0;
               --  begin
               case Current_State is
                  when Searching =>
                     Respond_To_Message_Searching;
                  when Coordinator =>
                     Respond_To_Message_Coordinator;
                  when Waiting_For_Turn =>
                     Respond_To_Message_Waiting_For_Turn;
                  when Approaching_Globe =>
                     Respond_To_Message_Approaching_Globe;
               end case;
                  --  if Iterations > Max_Mailbox_Checks then
                  --     exit;
                  --  end if;
                  --  Iterations := Iterations + 1;
               --  end;
            end loop;

            -- message sending / state
            case Current_State is
               when Searching =>
                  if not (Energy_Globes_Around'Length = 0) then
                     -- find first Globe
                     Target_Globe_Pos := Find_Closest_Globe (Energy_Globes_Around);
                     Send ((Purpose   => Broadcast_Globe_Pos,
                            Sender_No => Vehicle_No,
                            Target_No => Vehicle_No,
                            Charge    => Current_Charge,
                            Globe_Pos => Target_Globe_Pos,
                            Forward_Count => 0
                           ));
                     Set_Destination (Target_Globe_Pos);
                     Current_State := Coordinator;
                     Coordinator_No := Vehicle_No;
                     Waiting_Vehicles.Clear;
                     Debug_Print (Positive'Image (Vehicle_No) & " found Globe, became Coordinator");
                     --  Put_Line ("First Globe at: " & Real'Image (Target_Globe_Pos (x)) & Real'Image (Target_Globe_Pos (y)) & Real'Image (Target_Globe_Pos (z)));
                  end if;
               when Coordinator =>
                  if not (Energy_Globes_Around'Length = 0) then
                     -- update Globe pos
                     Target_Globe_Pos := Find_Closest_Globe (Energy_Globes_Around);
                     Send ((Purpose   => Broadcast_Globe_Pos,
                            Sender_No => Vehicle_No,
                            Target_No => Vehicle_No,
                            Charge    => Current_Charge,
                            Globe_Pos => Target_Globe_Pos,
                            Forward_Count => 0
                           ));
                     --  Put_Line ("Globe at: " & Real'Image (Target_Globe_Pos (x)) & Real'Image (Target_Globe_Pos (y)) & Real'Image (Target_Globe_Pos (z)));
                  end if;

                  if Pause_Duration < 1 then
                     if Waiting_Vehicles.Is_Empty then
                        --  Put_Line ("ERROR - no vehicles waiting");
                        null;
                     else
                        Lowest_Charge_No := Min_Map_Charge (Waiting_Vehicles);
                        if not (Lowest_Charge_No = Vehicle_No) then
                           Waiting_Vehicles.Exclude (Lowest_Charge_No);
                           --  Wait_Duration := Release_Delay;
                           Send ((Purpose   => Release,
                                  Sender_No => Vehicle_No,
                                  Target_No => Lowest_Charge_No,
                                  Charge    => Current_Charge,
                                  Globe_Pos => Target_Globe_Pos,
                                  Forward_Count => 0
                                 ));
                           --  Set_Destination ((Target_Globe_Pos (x), Target_Globe_Pos (y) + 0.5, Target_Globe_Pos (z)));
                           --  Set_Throttle (1.0);
                           --  delay (0.5);
                           Debug_Print (Positive'Image (Vehicle_No) &
                                          " Coordinator attemped to release " &
                                          Positive'Image (Lowest_Charge_No));
                        end if;
                     end if;
                     Pause_Duration := Release_Delay;
                  else
                     Pause_Duration := Pause_Duration - 1;
                  end if;
               when Waiting_For_Turn =>
                  if Current_Charge < 0.2 then
                     Current_State := Approaching_Globe;
                     Debug_Print (Positive'Image (Vehicle_No) &
                                    " approaching Globe due to critical charge");
                  elsif Pause_Duration < 1 then
                     Send ((Purpose   => Notify_Of_Charge,
                            Sender_No => Vehicle_No,
                            Target_No => Coordinator_No,
                            Charge    => Current_Charge,
                            Forward_Count => 0
                           ));
                     Pause_Duration := Notify_Delay;
                  else
                     Pause_Duration := Pause_Duration - 1;
                  end if;
                  null;
               when others =>
                  --  Put_Line (Real'Image (Target_Globe_Pos (x)) & Real'Image (Target_Globe_Pos (y)) & Real'Image (Target_Globe_Pos (z)));
                  null;
            end case;

            -- movement
            case Current_State is
               when Coordinator =>
                  if Calculate_Distance_To_Pos (Target_Globe_Pos) < Coordinator_Dist then
                     Set_Throttle (0.0);
                  else
                     Set_Destination (Target_Globe_Pos);
                     Set_Throttle (0.5);
                  end if;
               when Waiting_For_Turn =>
                  if Calculate_Distance_To_Pos (Target_Globe_Pos) < Waiting_Dist then
                     Set_Throttle (0.0);
                  else
                     Set_Destination (Target_Globe_Pos);
                     Set_Throttle (0.5);
                  end if;
               when Approaching_Globe =>
                  Set_Destination (Target_Globe_Pos);
                  if Calculate_Distance_To_Pos (Target_Globe_Pos) > 0.05 then
                     Set_Throttle (1.0);
                  else
                     Set_Throttle (0.0);
                  end if;
                  if Current_Charge > 0.8 then
                     Send ((Purpose   => Notify_Of_Charge,
                            Sender_No => Vehicle_No,
                            Target_No => Coordinator_No,
                            Charge    => Current_Charge,
                            Forward_Count => 0
                           ));
                     Current_State := Waiting_For_Turn;
                     Debug_Print (Positive'Image (Vehicle_No) & " finished travelling to Globe");
                     Set_Throttle (0.5);
                  end if;
               when others =>
                  null;
            end case;

         end loop Outer_task_loop;

      end select;

   exception
      when E : others => Show_Exception (E);

   end Vehicle_Task;

end Vehicle_Task_Type;
