# Paddlr

A canoe paddling and portage tracker. Designed to automate routes through unknown waterways.

## Kiosk

The front end, client facing application.

React, leaflet, axios

## Cartographer

The back end api.

Rust, Rocket, GeoJSON

### Notes
Speeding up the pathfinding could be accomplished via a bounding box before CDT. This would greatly reduce the number of triangles computed and speed up A*.
The one caveat is that the start and end nodes could be located in two different polygons.
Once a bounding box divides the polygons, we can check each polygon to see if it contains a point. We can also apply a filter, as the points will exist in either corners of the bounding box.
If both points exist in one polygon, the focus polygon