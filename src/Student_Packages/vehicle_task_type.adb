-- Suggestions for packages which might be useful:

--  with Ada.Real_Time;              use Ada.Real_Time;
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

   task body Vehicle_Task is

      Vehicle_No        : Positive;
      CurrentState      : State := Searching;
      LastMessage       : Inter_Vehicle_Messages;
      TargetGlobePos    : Positions;
      IsLowestCharge    : Boolean := False;

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

            if Messages_Waiting then
               Receive (LastMessage);
               if LastMessage.Purpose = NotifyOfGlobe and then CurrentState = Searching then
                  TargetGlobePos := LastMessage.GlobePos;
                  IsLowestCharge := Current_Charge < LastMessage.Charge;
                  Send ((NotifyOfGlobe, TargetGlobePos, Current_Charge)); -- forward
                  CurrentState := WaitingForTurn;
               elsif CurrentState = WaitingForTurn then
                  IsLowestCharge := Current_Charge < LastMessage.Charge;
                  if IsLowestCharge then
                     Send ((NotifyOfCharge, TargetGlobePos, Current_Charge)); -- forward
                  end if;
                  if LastMessage.Purpose = NotifyOfGlobe then -- update target globe
                     TargetGlobePos := LastMessage.GlobePos;
                     Send ((NotifyOfGlobe, TargetGlobePos, Current_Charge)); -- forward
                  end if;
               end if;
            elsif CurrentState = Searching and then not (Energy_Globes_Around'Length = 0) then
               --  Set_Throttle (0.1);
               --  Set_Destination (TargetGlobePos);
               TargetGlobePos := Find_Closest_Globe (Energy_Globes_Around);
               Send ((NotifyOfGlobe, TargetGlobePos, Current_Charge));
               CurrentState := WaitingForTurn;
            elsif CurrentState = WaitingForTurn then
               if IsLowestCharge and then Current_Charge < 0.9 then
                  Set_Throttle (1.0);
                  Set_Destination (TargetGlobePos);
                  CurrentState := ApproachingGlobe;
                  IsLowestCharge := True;
                  Put_Line (Positive'Image (Vehicle_No) & " Travelling to Globe");
               else
                  Send ((NotifyOfCharge, TargetGlobePos, Current_Charge));
               end if;
            elsif CurrentState = ApproachingGlobe and then Current_Charge > 0.9 then
               Set_Throttle (0.5);
               CurrentState := Searching;
               Put_Line (Positive'Image (Vehicle_No) & " Finished travelling to Globe");
               Send ((NotifyOfCharge, TargetGlobePos, Current_Charge));
               --  delay 1.0;
            end if;
         end loop Outer_task_loop;

      end select;

   exception
      when E : others => Show_Exception (E);

   end Vehicle_Task;

end Vehicle_Task_Type;
