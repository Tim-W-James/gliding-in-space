with GLOBE_3D;

package Sierpinski_P is

  type Cubic_Face_count is new Integer range 1 .. 6;
  type Cubic_Face_texture is array (Cubic_Face_count) of GLOBE_3D.Image_ID;

  procedure Create (
    Object        : in out GLOBE_3D.p_Object_3D;
    Scale         :        GLOBE_3D.Real;
    Centre        :        GLOBE_3D.Point_3D;
    texture       :        Cubic_Face_texture := (others => GLOBE_3D.null_image);
    tiled         :        Boolean := True;
    fractal_level :        Natural := 2
);

end Sierpinski_P;
