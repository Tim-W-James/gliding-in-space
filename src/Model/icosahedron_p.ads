with GLOBE_3D;

package Icosahedron_P is

  procedure Create (
    Object : in out GLOBE_3D.p_Object_3D;
    Scale  :        GLOBE_3D.Real;
    Centre :        GLOBE_3D.Point_3D;
    alpha  :        GLOBE_3D.Real := 1.0;
    polyball :      Boolean := False
  );

end Icosahedron_P;
