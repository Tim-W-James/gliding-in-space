------------------------------------------------------------------------------
--  File :            X29.adb
--  Description :     x29 aircraft data base from Evans & Sutherland
--  Copyright : (c) Evans & Sutherland -- ok to distribute if copyright appears
------------------------------------------------------------------------------

with GLOBE_3D;

package X29_P is
  procedure Create (
    Object : in out GLOBE_3D.p_Object_3D;
    Scale : GLOBE_3D.Real;
    Centre : GLOBE_3D.Point_3D
  );
end X29_P;
