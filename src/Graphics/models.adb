--
-- Jan & Uwe R. Zimmer, Australia, July 2011
--

with Ada.Environment_Variables;

-- NOTE: Why are we elaborating? What's to elaborate?
with Sphere_P; pragma Elaborate_All (Sphere_P);

with A319_P;
with Arrow_P;
with Box_P;
with Cube_P;
with Dreadnought_P;
with Duck_P;
with Icosahedron_P;
with Lissajous_P;
with Plane_P;
with Sierpinski_P;
with SkotKnot_P;
with Spaceship_P;
with Vehic001_P;
with Vehic002_P;
with X29_P;

with Real_Type; use Real_Type;
with Vectors_4D; use Vectors_4D;

with GL;
with GL.Materials; use GL.Materials;

package body Models is
   procedure Assign_Material (Model : GLOBE_3D.p_Object_3D; Material : Material_type) is
   begin
      for Faces in Model.all.face'Range loop
         Model.all.face (Faces).material := Material;
      end loop;
   end Assign_Material;

   function To_Vector_4D (V : GL.Material_Float_vector) return Vector_4D is
     (x => Real (V (0)),
      y => Real (V (1)),
      z => Real (V (2)),
      t => Real (V (3)));

   function To_GL (V : Vector_4D) return GL.Material_Float_vector is
     (0 => GL.C_Float (V (x)),
      1 => GL.C_Float (V (y)),
      2 => GL.C_Float (V (z)),
      3 => GL.C_Float (V (t)));

   subtype Ratio_T is Real range 0.0 .. 1.0;

   function Configured_Model return Model_Name is
      Env_Name : constant String := "SWARM_MODEL";
      Default_Model : constant Model_Name := Models.Spaceship;
   begin
      if Ada.Environment_Variables.Exists (Env_Name) then
         declare
            Name : constant String := Ada.Environment_Variables.Value (Env_Name);
         begin
            if Name'Length = 0 then
               return Default_Model;
            else
               return Model_Name'Value (Name);
            end if;
         end;
      else
         return Default_Model;
      end if;
   end Configured_Model;

   function Blend_Material (Material_1, Material_2 : Material_type; Ratio : Ratio_T) return Material_type is
     (ambient   => To_GL (Ratio * To_Vector_4D (Material_1.ambient)  + (1.0 - Ratio) *  To_Vector_4D (Material_2.ambient)),
      diffuse   => To_GL (Ratio * To_Vector_4D (Material_1.diffuse)  + (1.0 - Ratio) *  To_Vector_4D (Material_2.diffuse)),
      specular  => To_GL (Ratio * To_Vector_4D (Material_1.specular) + (1.0 - Ratio) *  To_Vector_4D (Material_2.specular)),
      emission  => To_GL (Ratio * To_Vector_4D (Material_1.emission) + (1.0 - Ratio) *  To_Vector_4D (Material_2.emission)),
      shininess => GL.C_Float (Ratio * Real (Material_1.shininess) + (1.0 - Ratio) * Real (Material_2.shininess)));

   procedure Initialize is
      Model_Kind : constant Model_Name := Configured_Model;
      Model_Object : GLOBE_3D.p_Object_3D;
   begin
      -- Sphere model is used for energy globes
      Sphere_P.Create (Object => Model_Set (Sphere), Scale => 0.015, Centre => (0.0, 0.0, 0.0));
      Assign_Material (Model_Set (Sphere), Ruby);

      case Model_Kind is
         when A319 =>        A319_P.Create        (Object => Model_Object, Scale  => 0.002, Centre => (0.0, 0.0, 0.0));
         when Arrow =>       Arrow_P.Create       (Object => Model_Object, Scale  => 0.004, Centre => (0.0, 0.0, 0.0));
         when Box =>         Box_P.Create         (Object => Model_Object, Scale  => 0.015, Centre => (0.0, 0.0, 0.0));
         when Cube =>        Cube_P.Create        (Object => Model_Object, Scale  => 0.015, Centre => (0.0, 0.0, 0.0));
         when Dreadnought => Dreadnought_P.Create (Object => Model_Object, Scale  => 0.0000025, Centre => (0.0, 0.0, 0.0));
         when Duck =>        Duck_P.Create        (Object => Model_Object, Scale  => 0.001, Centre => (0.0, 0.0, 0.0));
         when Icosahedron => Icosahedron_P.Create (Object => Model_Object, Scale  => 0.01, Centre => (0.0, 0.0, 0.0));
         when Lissajous =>   Lissajous_P.Create   (Object => Model_Object, Scale  => 0.002, Centre => (0.0, 0.0, 0.0));
         when Plane =>       Plane_P.Create       (Object => Model_Object, Scale  => 0.001, Centre => (0.0, 0.0, 0.0));
         when Sierpinski =>  Sierpinski_P.Create  (Object => Model_Object, Scale  => 0.02, Centre => (0.0, 0.0, 0.0));
         when SkotKnot =>    SkotKnot_P.Create    (Object => Model_Object, Scale  => 0.0025, Centre => (0.0, 0.0, 0.0));
         when Spaceship =>   Spaceship_P.Create   (Object => Model_Object, Scale  => 0.003, Centre => (0.0, 0.0, 0.0));
         when Sphere =>      Sphere_P.Create      (Object => Model_Object, Scale  => 0.01, Centre => (0.0, 0.0, 0.0));
         when Vehic001 =>    Vehic001_P.Create    (Object => Model_Object, Scale  => 0.002, Centre => (0.0, 0.0, 0.0));
         when Vehic002 =>    Vehic002_P.Create    (Object => Model_Object, Scale  => 0.02, Centre => (0.0, 0.0, 0.0));
         when X29 =>         X29_P.Create         (Object => Model_Object, Scale  => 0.004, Centre => (0.0, 0.0, 0.0));
      end case;

      for M in Spaceship_Gradient'Range (1) loop
         for i in Spaceship_Gradient'Range (2) loop
            declare
               Model_Copy : constant GLOBE_3D.p_Object_3D := new GLOBE_3D.Object_3D (Model_Object.Max_points, Model_Object.Max_faces);
            begin
               Model_Copy.all := Model_Object.all;
               Spaceship_Gradient (M, i) := Model_Copy;
            end;

            declare
               Ratio : constant Ratio_T :=
                 ((Real (i) - Real (Spaceship_Gradient'First (2)))
                  / Real (Spaceship_Gradient'Last (2) - Spaceship_Gradient'First (2)))
                 + Ratio_T'First;
            begin
               case M is
                  when G_Ruby => Assign_Material (Spaceship_Gradient (M, i), Blend_Material (Ruby, Pearl, Ratio));
                  when G_Turquoise => Assign_Material (Spaceship_Gradient (M, i), Blend_Material (Turquoise, Pearl, Ratio));
               end case;
            end;
         end loop;
      end loop;
   end Initialize;

begin
   Initialize;
end Models;
