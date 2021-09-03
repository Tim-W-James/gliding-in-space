--
 -- Jan & Uwe R. Zimmer, Australia, July 2011
 --

with GLOBE_3D;

package Arrow_P is

   procedure Create (Object       : in out GLOBE_3D.p_Object_3D;
                     Scale :        GLOBE_3D.Real;
                     Centre       :        GLOBE_3D.Point_3D);

end Arrow_P;
