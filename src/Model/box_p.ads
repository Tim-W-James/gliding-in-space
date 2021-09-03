with GLOBE_3D;

package Box_P is

  procedure Create (Object   : in out GLOBE_3D.p_Object_3D;
                    Sides    :        GLOBE_3D.Vector_3D := (1.0, 1.0, 1.0);
                    Scale    :        GLOBE_3D.Real      := 1.0;
                    Centre   :        GLOBE_3D.Point_3D  := (0.0, 0.0, 0.0));

end Box_P;
