-- name: drop_merged
drop table if exists merged;

-- name: drop_holes
drop table if exists holes;

-- name: drop_focus_polygon
drop table if exists focus_polygon;

-- name: merged
CREATE TABLE merged as
 with point_geom as (
 	select wkb_geometry from points
 ),
 --envelope to only union what is inside of the bounding box
 envelope as (
 	select ST_Extent(point_geom.wkb_geometry) as geom from point_geom
 )
 SELECT ST_Union(way) as tunion
 FROM planet_osm_polygon, envelope
 where "natural" = 'water' and (way && envelope.geom);
 
-- name: focus_polygon
create table focus_polygon as
with polygons as (
	select (st_dump(tunion)).geom as polygon
	from merged
),
pointw as (
	select wkb_geometry from points where id = (select MAX(id) from points)
)
select polygon
from polygons, pointw
where st_within(pointw.wkb_geometry, polygons.polygon);

-- name: holes
-- Get all interior rings (holes) plus the exterior ring
-- The geom field contains each ring as a POLYGON. The path field is an integer array of length 1 containing the polygon ring index. The exterior ring (shell) has index 0. The interior rings (holes) have indices of 1 and higher.
create table holes as
SELECT (ring).path, (ring).geom
FROM (
  SELECT ST_DumpRings(polygon) AS ring
  FROM focus_polygon
) AS sub;


