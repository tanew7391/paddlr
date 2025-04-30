--CREATE EXTENSION postgis;
--CREATE EXTENSION pgrouting;
--CREATE EXTENSION postgis_sfcgal;

drop table if exists merged; 
drop table if exists median_axis ;
drop table if exists start_end_keys;
drop table if exists median_axis_disjoint ;
drop table if exists median_axis_disjoint_noded;
drop table if exists median_axis_disjoint_noded_vertices_pgr;
drop table if exists median_axis_not_conformed ;
drop table if exists output;

CREATE TABLE merged as
 with pointa as (
 	select wkb_geometry from points where ogc_fid = 1
 ),
 pointb as (
 	select wkb_geometry from points where ogc_fid = 2
 ),
 --envelope to only union what is inside of the bounding box
 envelope as (
 	select ST_MakeEnvelope(ST_X(pointb.wkb_geometry), ST_Y(pointb.wkb_geometry), ST_X(pointa.wkb_geometry), ST_Y(pointa.wkb_geometry)) as geom from pointa, pointb
 )
 SELECT ST_Union(way) as tunion
 FROM planet_osm_polygon, envelope
 where "natural" = 'water' and (way && envelope.geom);
 
create temp table focus_polygon as
with polygons as (
	select (st_dump(tunion)).geom as polygon
	from merged
),
pointw as (
	select wkb_geometry from points where ogc_fid = 2
)
select polygon
from polygons, pointw
where st_within(pointw.wkb_geometry, polygons.polygon);


--Slow, restrict by bounding box
create table median_axis as 
select ST_ApproximateMedialAxis(
				ST_SimplifyPreserveTopology(
				(select * from focus_polygon), 30.0)
			) as geom;
 
-- Create a table of disjoint lines belonging to polygon that houses the start point (ogc_fid=2)
--Very long query!!!
create table median_axis_disjoint as
select geom.geom as the_geom
from st_dump(
			(select geom
			from median_axis)
		) as geom;




ALTER TABLE median_axis_disjoint 
ADD COLUMN id INTEGER PRIMARY KEY GENERATED ALWAYS AS IDENTITY;

CREATE TYPE start_end_enum AS ENUM ('START', 'END');

CREATE TABLE start_end_keys (
    id SERIAL PRIMARY KEY,
    state start_end_enum NOT NULL UNIQUE, -- Ensure only one of each value (START or END)
    median_axis_disjoint_id INT NOT NULL,
    CONSTRAINT fk_median_axis_disjoint
        FOREIGN KEY (median_axis_disjoint_id) REFERENCES median_axis_disjoint(id),
    CONSTRAINT unique_state UNIQUE (state)
);

with rows as  (
insert into median_axis_disjoint (the_geom)
	(SELECT ST_MakeLine(i.wkb_geometry, ST_ClosestPoint(ma.geom, i.wkb_geometry)) AS geom
    FROM (
		select ogc_fid, wkb_geometry from points where ogc_fid = 1
	) as i
    INNER JOIN (
        SELECT ST_Collect(geom) AS geom
             FROM median_axis
    ) as ma
    ON i.ogc_fid = 1)
  returning id
) insert into start_end_keys (state, median_axis_disjoint_id)
	select t.i, r.id
	from rows r
	cross join (values('START'::start_end_enum)) t(i);
   
with rows as  (
insert into median_axis_disjoint (the_geom)
	--Create a line from the selected end point to the closest point on the skeleton and insert into the skeleton
	(SELECT ST_MakeLine(i.wkb_geometry, ST_ClosestPoint(ma.geom, i.wkb_geometry)) AS geom
    FROM (
		select ogc_fid, wkb_geometry from points where ogc_fid = 2
	) as i
    INNER JOIN (
        SELECT ST_Collect(geom) AS geom
             FROM median_axis
    ) as ma
    ON i.ogc_fid = 2)
  returning id
) insert into start_end_keys (state, median_axis_disjoint_id)
	select t.i, r.id
	from rows r
	cross join (values('END'::start_end_enum)) t(i);
			

SELECT pgr_nodeNetwork('median_axis_disjoint', .0001);

ALTER TABLE median_axis_disjoint_noded
    ADD cost NUMERIC
    GENERATED ALWAYS AS (
        ST_Length(the_geom)
    )
    STORED
;

ALTER TABLE median_axis_disjoint_noded
	ADD reverse_cost NUMERIC NOT NULL DEFAULT -1
;

UPDATE median_axis_disjoint_noded n
    SET reverse_cost = ST_Length(n.the_geom)
    FROM median_axis_disjoint r
    WHERE n.old_id = r.id
;

SELECT pgr_createTopology('median_axis_disjoint_noded', 0.0001);
SELECT pgr_analyzeGraph('median_axis_disjoint_noded', 0.0001);

create table output as
SELECT seq, the_geom
    FROM pgr_dijkstra('
        SELECT rn.id, rn.source,
              rn.target, rn.cost,
              rn.reverse_cost
            FROM median_axis_disjoint_noded rn
            WHERE source IS NOT NULL
          ',
         (select "source" from median_axis_disjoint_noded where old_id = (
         	select median_axis_disjoint_id
         	from start_end_keys
         	where state = 'START'::start_end_enum
         )), -- start node
         (select "source" from median_axis_disjoint_noded where old_id = (
         	select median_axis_disjoint_id
         	from start_end_keys
         	where state = 'END'::start_end_enum
         )), -- end node
         True -- Enforce one-way restrictions
        ) as d
    INNER JOIN median_axis_disjoint_noded x
        ON d.edge = x.id
;
