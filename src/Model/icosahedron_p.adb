pragma Warnings (Off);
pragma Style_Checks (Off);

with GL;

package body Icosahedron_P is

  procedure Create (
    Object : in out GLOBE_3D.p_Object_3D;
    Scale  :        GLOBE_3D.Real;
    Centre :        GLOBE_3D.Point_3D;
    alpha  :        GLOBE_3D.Real := 1.0;
    polyball :      Boolean := False
)
  is

    use GL, GLOBE_3D;

    nb_points : constant := 12;
    nb_faces :  constant := 20;

    obj_points : constant array (1 .. nb_points, Point_3D'Range) of Real :=
       ((0.0,        0.0,       2.0),
                (1.78885,    0.0,       0.894427),
                (0.552786,   1.7013,    0.894427),
                ( - 1.44721,    1.05146,   0.894427),
                ( - 1.44721,   - 1.05146,   0.894427),
                (0.552786,  - 1.7013,    0.894427),
                (1.44721,    1.05146,  - 0.894427),
                ( - 0.552786,   1.7013,   - 0.894427),
                ( - 1.78885,    0.0,      - 0.894427),
                ( - 0.552786,  - 1.7013,   - 0.894427),
                (1.44721,   - 1.05146,  - 0.894427),
                (0.0,        0.0,      - 2.0));

    obj_faces : constant array (1 .. nb_faces, 1 .. 3) of Positive :=
       ((3,  1,  2),
          (4,  1,  3),
          (5,  1,  4),
          (6,  1,  5),
          (2,  1,  6),
          (3,  2,  7),
          (8,  3,  7),
          (4,  3,  8),
          (9,  4,  8),
          (5,  4,  9),
          (10,  5,  9),
          (6,  5,  10),
          (11,  6,  10),
          (7,  2,  11),
          (2,  6,  11),
          (7, 12,   8),
          (8, 12,   9),
          (9, 12,  10),
          (10, 12,  11),
          (11, 12,   7));

    colour  : GL.RGB_Color;
    face_0  : Face_type; -- takes defaults values
  begin
    face_0.skin    := colour_only;
    face_0.alpha   := alpha;

    if polyball then
      Object := new Object_3D (nb_points*nb_points, nb_points*nb_faces);
    else
      Object := new Object_3D (Max_points => nb_points, Max_faces => nb_faces);
    end if;

    for i in 1 .. nb_points loop
      for d in Point_3D'Range loop
        if polyball then
          for j in 1 .. nb_points loop
            Object.point ((i - 1)*nb_points + j) (d) := Scale * (obj_points (i, d) + obj_points (j, d)*0.3);
          end loop;
        else
          Object.point (i) (d) := Scale * obj_points (i, d);
        end if;
      end loop;
    end loop;

    for i in 1 .. nb_faces loop
      case i mod 10 is
        when 0 => colour := (0.1, 0.1, 0.1);
        when 1 => colour := (0.1, 0.1, 1.0);
        when 2 => colour := (0.1, 1.0, 0.1);
        when 3 => colour := (0.1, 1.0, 1.0);
        when 4 => colour := (1.0, 0.1, 0.1);
        when 5 => colour := (1.0, 0.1, 1.0);
        when 6 => colour := (1.0, 1.0, 0.1);
        when 7 => colour := (1.0, 1.0, 1.0);
        when 8 => colour := (0.3, 0.3, 0.3);
        when 9 => colour := (0.6, 0.6, 0.6);
        when others => null;
      end case;

      face_0.colour  := colour;
      face_0.P := (obj_faces (i, 1), obj_faces (i, 2), obj_faces (i, 3), 0);
      Object.face (i) := face_0;
    end loop;
    if polyball then
      for i in 1 .. nb_points - 1 loop
        for j in 1 .. nb_faces loop
          declare
            f : Face_type renames Object.face (i*nb_faces + j);
          begin
            f := Object.face (j);
            for pt in f.P'Range loop
              if f.P (pt) /= 0 then
                f.P (pt) := f.P (pt) + i*nb_points;
              end if;
            end loop;
          end;
        end loop;
      end loop;
    end if;

    Object.Centre := Centre;
    Set_name (Object.all, "icosahedron");
  end Create;

end Icosahedron_P;
