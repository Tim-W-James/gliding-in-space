-- * Output of max2ada.ms, a GMax / 3D Studio Max script for exporting to GLOBE_3D
--
-- * Copy and paste these lines from the Listener into a
--   text editor, and save the package as an .ada file.
-- * Alternatively, use the GMaxSLGRAB.exe tool.
-- * For GNAT, you must save the specification as an .ads file
--   and the body as an .adb file, or run gnatchop on the whole .ada file.

-- Title : Dreadnought
-- Subject : Dreadnought class Heavy Cruiser : a space cruiser inspired by Sulaco from Aliens.
-- Author : Xenodroid

-- File name : dreadnought_g3d.gmax
-- File path : C:\Ada\GL\3dmodels\dreadnought\

with GLOBE_3D;

package Dreadnought_P is
   procedure Create (
      Object : in out GLOBE_3D.p_Object_3D;
      Scale  :        GLOBE_3D.Real;
      Centre :        GLOBE_3D.Point_3D;
      alum_001,
      grumnoir,
      tole_001,
      alum_002 : GLOBE_3D.Image_ID := GLOBE_3D.null_image
   );
end Dreadnought_P;
