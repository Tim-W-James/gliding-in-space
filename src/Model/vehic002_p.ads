pragma Warnings (Off);
pragma Style_Checks (Off);

------------------------------------------------------------------------------
 --  File :            Vehic002.ads
 --  Description :     3D model of a space vehicle. Big, mysterious ship.
 --                   Copyright (c) Gautier de Montmollin 1999 - 2000
 ------------------------------------------------------------------------------

with GLOBE_3D;

package Vehic002_P is
  procedure Create (
    Object : in out GLOBE_3D.p_Object_3D;
    Scale  :        GLOBE_3D.Real;
    Centre :        GLOBE_3D.Point_3D;
    metal_door,
    metal_surface,
    bumped_blue   : GLOBE_3D.Image_id := GLOBE_3D.null_image
  );
end Vehic002_P;
