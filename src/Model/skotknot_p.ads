pragma Warnings (Off);
pragma Style_Checks (Off);

-- VRML : [#VRML V1.0 ascii
 -- ]
with GLOBE_3D;

package SkotKnot_P is
  procedure Create (
    Object : in out GLOBE_3D.p_Object_3D;
    Scale  :        GLOBE_3D.Real;
    Centre :        GLOBE_3D.Point_3D
  );
end SkotKnot_P;
