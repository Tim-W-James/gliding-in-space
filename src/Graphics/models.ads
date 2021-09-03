--
-- Jan & Uwe R. Zimmer, Australia, July 2011
--

with GLOBE_3D;

package Models is

   pragma Elaborate_Body;
   type Model_Name is (
      A319,
      Arrow,
      Box,
      Cube,
      Dreadnought,
      Duck,
      Icosahedron,
      Lissajous,
      Plane,
      Sierpinski,
      SkotKnot,
      Spaceship,
      Sphere,
      Vehic001,
      Vehic002,
      X29
   );

   type Gradient_Materials is (G_Turquoise, G_Ruby);

   type Models_Field   is array (Model_Name)                                     of GLOBE_3D.p_Object_3D;
   type Gradient_Field is array (Gradient_Materials range <>, Positive range <>) of GLOBE_3D.p_Object_3D;

   Model_Set : Models_Field;

   Spaceship_Gradient : Gradient_Field (Gradient_Materials'Range, 1 .. 10);

end Models;
