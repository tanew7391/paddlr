CREATE TABLE public.points (
	id serial4 NOT NULL,
	wkb_geometry public.geometry(point, 3857) NULL,
	CONSTRAINT points_pkey PRIMARY KEY (id)
);
CREATE INDEX points_wkb_geometry_geom_idx ON public.points USING gist (wkb_geometry);