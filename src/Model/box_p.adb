pragma Style_Checks (Off);
pragma Warnings (Off);

with GL;
with GLOBE_3D.Textures;

package body Box_P is
   package g3d renames GLOBE_3D;

   procedure Create (Object   : in out GLOBE_3D.p_Object_3D;
                     Sides    :        GLOBE_3D.Vector_3D := (1.0, 1.0, 1.0);
                     Scale    :        GLOBE_3D.Real      := 1.0;
                     Centre   :        GLOBE_3D.Point_3D  := (0.0, 0.0, 0.0))
   is
      use GLOBE_3D, GL, GLOBE_3D.REF;

      function Basic_face (P       : G3D.Natural_Index_array;
                           colour  : GL.RGB_Color;
                           repeat  : Positive)       return Face_type
      is
         f : Face_type; -- takes defaults values
         alpha  : GL.Double := 1.0;
         new_id : Image_ID;
      begin
         f.P       := P;
         f.skin    := material_only;
         f.texture := null_image;
         f.colour  := colour;
         f.alpha   := alpha;
         f.repeat_U := repeat;
         f.repeat_V := repeat;
         return f;
      end Basic_face;

      half_Width   : constant g3d.Real := Scale * Sides (0) / 2.0;
      half_Height  : constant g3d.Real := Scale * Sides (1) / 2.0;
      half_Depth   : constant g3d.Real := Scale * Sides (2) / 2.0;

   begin

      Object := new G3D.Object_3D (Max_points => 8,  Max_faces => 6);

      Object.Centre := Centre;

      Object.point := (( - half_Width, - half_Height, - half_Depth),
                      ( - half_Width,  half_Height, - half_Depth),
                      (half_Width,  half_Height, - half_Depth),
                      (half_Width, - half_Height, - half_Depth),
                      ( - half_Width, - half_Height,  half_Depth),
                      ( - half_Width,  half_Height,  half_Depth),
                      (half_Width,  half_Height,  half_Depth),
                      (half_Width, - half_Height,  half_Depth));

      Object.face := (Basic_face ((3, 2, 6, 7), (1.0, 0.0, 0.0), 1),
                     Basic_face ((4, 3, 7, 8), (0.0, 1.0, 0.0), 2),
                     Basic_face ((8, 7, 6, 5), (0.0, 0.0, 1.0), 3),
                     Basic_face ((1, 4, 8, 5), (1.0, 1.0, 0.0), 4),
                     Basic_face ((2, 1, 5, 6), (0.0, 1.0, 1.0), 5),
                     Basic_face ((3, 4, 1, 2), (1.0, 0.0, 1.0), 6));

      Set_name (Object.all, "a box");
  end Create;
end Box_P;
