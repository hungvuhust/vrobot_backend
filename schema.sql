-- Schema public
CREATE SCHEMA IF NOT EXISTS public;

-- Bảng bản đồ
CREATE TABLE IF NOT EXISTS public.Map (
    id_map SERIAL PRIMARY KEY,
    map_name TEXT UNIQUE NOT NULL,
    width INTEGER NOT NULL,
    height INTEGER NOT NULL,
    resolution REAL NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    theta REAL NOT NULL,
    image TEXT NOT NULL
);

-- Bảng Node
CREATE TABLE IF NOT EXISTS public.Node (
    id SERIAL PRIMARY KEY,
    x REAL NOT NULL,
    y REAL NOT NULL,
    theta REAL NOT NULL,
    type TEXT NOT NULL,
    map_id INTEGER NOT NULL,
    FOREIGN KEY (map_id) REFERENCES public.Map(id_map) ON DELETE CASCADE
);

-- Bảng đường thẳng
CREATE TABLE IF NOT EXISTS public.StraightLink (
    id_straight_link SERIAL PRIMARY KEY,
    id_start INTEGER NOT NULL,
    id_stop INTEGER NOT NULL,
    map_id INTEGER NOT NULL,
    FOREIGN KEY (id_start) REFERENCES public.Node(id) ON DELETE CASCADE,
    FOREIGN KEY (id_stop)  REFERENCES public.Node(id) ON DELETE CASCADE,
    FOREIGN KEY (map_id)   REFERENCES public.Map(id_map) ON DELETE CASCADE
);

-- Bảng đường cong
CREATE TABLE IF NOT EXISTS public.CurveLink (
    id_curve_link SERIAL PRIMARY KEY,
    id_start INTEGER NOT NULL,
    id_stop INTEGER NOT NULL,
    control_point_1_x REAL NOT NULL,
    control_point_1_y REAL NOT NULL,
    control_point_2_x REAL NOT NULL,
    control_point_2_y REAL NOT NULL,
    map_id INTEGER NOT NULL,
    FOREIGN KEY (id_start) REFERENCES public.Node(id) ON DELETE CASCADE,
    FOREIGN KEY (id_stop)  REFERENCES public.Node(id) ON DELETE CASCADE,
    FOREIGN KEY (map_id)   REFERENCES public.Map(id_map) ON DELETE CASCADE
);

-- Cấp quyền cho PostgREST role
GRANT USAGE ON SCHEMA public TO vrobot;
GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO vrobot;